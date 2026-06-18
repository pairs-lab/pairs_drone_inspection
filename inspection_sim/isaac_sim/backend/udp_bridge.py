"""udp_bridge.py — UDP→MJPEG ffmpeg command builder and single-frame probe.

This module is intentionally free of FastAPI / asyncio so it can be imported
in plain unit tests without starting a server.

Functions
---------
build_mjpeg_cmd(host, port, quality, fps) -> list[str]
    Build the ffmpeg argv list that reads the UDP H.264/mpegts stream and
    writes a multipart-JPEG stream to stdout.

grab_single_jpeg(host, port, timeout_s) -> bytes | None
    Probe the UDP stream: capture exactly one JPEG frame and return its bytes.
    Returns None if the stream is not reachable within `timeout_s` seconds.
    Used by /api/live_status and unit tests — safe to call with no Isaac running.
"""

from __future__ import annotations

import subprocess
from typing import Optional

# ---------------------------------------------------------------------------
# Defaults (overridable via environment variables in app.py)
# ---------------------------------------------------------------------------
DEFAULT_UDP_HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 5600

# Timeout flags passed to ffmpeg so it doesn't hang forever when there is no
# UDP source.  udp_timeout is in *microseconds* (ffmpeg convention).
_UDP_TIMEOUT_US = 10_000_000   # 10 s


def _udp_input_url(host: str, port: int) -> str:
    return (
        f"udp://{host}:{port}"
        f"?fifo_size=1000000&overrun_nonfatal=1&timeout={_UDP_TIMEOUT_US}"
    )


# ---------------------------------------------------------------------------
# Command builders
# ---------------------------------------------------------------------------

def build_mjpeg_cmd(
    host: str = DEFAULT_UDP_HOST,
    port: int = DEFAULT_UDP_PORT,
    quality: int = 5,
    fps: int = 15,
) -> list[str]:
    """Return ffmpeg argv that reads UDP H.264/mpegts and emits mpjpeg to stdout.

    The multipart boundary produced by ``-f mpjpeg`` is ``ffmpeg``, so the
    correct Content-Type header is::

        multipart/x-mixed-replace; boundary=ffmpeg

    Args:
        host:    UDP source host (default 127.0.0.1).
        port:    UDP source port (default 5600).
        quality: JPEG quality scale 2–31, lower = better (default 5).
        fps:     Output frame rate (default 15).

    Returns:
        A list of strings suitable for ``subprocess.Popen(cmd, ...)``.
    """
    return [
        "ffmpeg",
        "-loglevel", "warning",
        # Allow 2 s of stream analysis so ffmpeg can locate an IDR keyframe
        # in the live mpegts/H.264 stream before producing the first JPEG.
        # Without this the H.264 decoder fails with "non-existing PPS" errors.
        "-analyzeduration", "2000000",
        "-probesize", "2000000",
        # Low-latency input flags
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        # UDP input with built-in timeout so we don't hang when no source
        "-i", _udp_input_url(host, port),
        # MJPEG output: multipart/x-mixed-replace (boundary="ffmpeg")
        "-f", "mpjpeg",
        "-q:v", str(quality),
        "-r", str(fps),
        # Write to stdout
        "-",
    ]


def build_single_frame_cmd(
    host: str = DEFAULT_UDP_HOST,
    port: int = DEFAULT_UDP_PORT,
    timeout_s: float = 10.0,
    out_path: str = "/tmp/udp_bridge_probe.jpg",
) -> list[str]:
    """Return ffmpeg argv that grabs exactly one JPEG frame to a file.

    Writes to ``out_path`` (a temp file) rather than stdout because the
    ``image2`` muxer needs seekable output; ``pipe:1`` causes ffmpeg to hang
    when the H.264 decoder emits PPS/SPS decode warnings before producing the
    first complete frame.

    Uses ``-analyzeduration 2000000 -probesize 2000000`` to allow ffmpeg to
    buffer enough of the live mpegts stream to locate an IDR (keyframe) before
    attempting to decode — without this the H.264 decoder fails with
    ``non-existing PPS referenced`` on mid-stream connections.
    """
    timeout_us = int(timeout_s * 1_000_000)
    udp_url = (
        f"udp://{host}:{port}"
        f"?fifo_size=1000000&overrun_nonfatal=1&timeout={timeout_us}"
    )
    return [
        "ffmpeg",
        "-loglevel", "warning",
        # Allow up to 2 s of stream analysis so ffmpeg finds an IDR keyframe
        # before decoding (avoids "non-existing PPS" errors on mid-stream connect)
        "-analyzeduration", "2000000",
        "-probesize", "2000000",
        "-i", udp_url,
        "-vframes", "1",
        "-f", "image2",
        "-update", "1",   # allow overwriting a single file (no sequence pattern needed)
        "-vcodec", "mjpeg",
        "-q:v", "5",
        "-y",             # overwrite without asking
        out_path,
    ]


# ---------------------------------------------------------------------------
# Probe helper
# ---------------------------------------------------------------------------

def grab_single_jpeg(
    host: str = DEFAULT_UDP_HOST,
    port: int = DEFAULT_UDP_PORT,
    timeout_s: float = 10.0,
) -> Optional[bytes]:
    """Capture one JPEG frame from the UDP stream.

    Writes to a temp file (not stdout) to avoid ``image2`` muxer seek errors.

    Args:
        host:      UDP host.
        port:      UDP port.
        timeout_s: How long to wait before giving up (wall-clock seconds).
                   This is also forwarded to ffmpeg's udp timeout flag.

    Returns:
        Raw JPEG bytes if successful, or ``None`` if the stream is not
        available (ffmpeg returned non-zero or produced no data).
    """
    import tempfile
    import os

    # Use a named temp file so ffmpeg can seek/overwrite it
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tf:
        out_path = tf.name

    cmd = build_single_frame_cmd(host=host, port=port, timeout_s=timeout_s, out_path=out_path)
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s + 5.0,   # extra headroom: analyzeduration adds ~2s before decode
        )
    except subprocess.TimeoutExpired:
        try:
            os.unlink(out_path)
        except OSError:
            pass
        return None
    except FileNotFoundError:
        # ffmpeg not installed
        return None

    # Read back the written file
    try:
        if not os.path.exists(out_path) or os.path.getsize(out_path) < 2:
            return None
        with open(out_path, "rb") as f:
            data = f.read()
    finally:
        try:
            os.unlink(out_path)
        except OSError:
            pass

    if result.returncode != 0 or not data:
        return None

    # Sanity: must start with JPEG SOI marker FF D8
    if len(data) < 2 or data[:2] != b"\xff\xd8":
        return None

    return data
