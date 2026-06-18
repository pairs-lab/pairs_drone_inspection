"""Tests for backend.udp_bridge — command builder and single-frame probe.

Pure unit tests: no server, no Isaac Sim, no live UDP stream required.
"""
import subprocess

import pytest

from backend.udp_bridge import (
    DEFAULT_UDP_HOST,
    DEFAULT_UDP_PORT,
    build_mjpeg_cmd,
    build_single_frame_cmd,
    grab_single_jpeg,
)


# ---------------------------------------------------------------------------
# build_mjpeg_cmd — structural / content tests
# ---------------------------------------------------------------------------

class TestBuildMjpegCmd:
    def test_returns_list_of_strings(self):
        cmd = build_mjpeg_cmd()
        assert isinstance(cmd, list)
        assert all(isinstance(s, str) for s in cmd)

    def test_starts_with_ffmpeg(self):
        cmd = build_mjpeg_cmd()
        assert cmd[0] == "ffmpeg"

    def test_contains_udp_url(self):
        cmd = build_mjpeg_cmd(host="127.0.0.1", port=5600)
        url_args = [a for a in cmd if a.startswith("udp://127.0.0.1:5600")]
        assert url_args, "No UDP URL found in command"

    def test_output_format_mpjpeg(self):
        cmd = build_mjpeg_cmd()
        assert "-f" in cmd
        idx = cmd.index("-f")
        assert cmd[idx + 1] == "mpjpeg"

    def test_output_target_is_dash(self):
        """Last argument must be '-' (stdout)."""
        cmd = build_mjpeg_cmd()
        assert cmd[-1] == "-"

    def test_custom_host_port(self):
        cmd = build_mjpeg_cmd(host="0.0.0.0", port=1234)
        url_args = [a for a in cmd if "udp://0.0.0.0:1234" in a]
        assert url_args, f"Expected UDP URL with custom host/port, got: {cmd}"

    def test_quality_flag(self):
        for q in (2, 5, 20):
            cmd = build_mjpeg_cmd(quality=q)
            assert "-q:v" in cmd
            idx = cmd.index("-q:v")
            assert cmd[idx + 1] == str(q)

    def test_fps_flag(self):
        for fps in (5, 15, 30):
            cmd = build_mjpeg_cmd(fps=fps)
            assert "-r" in cmd
            idx = cmd.index("-r")
            assert cmd[idx + 1] == str(fps)

    def test_low_latency_flags_present(self):
        cmd = build_mjpeg_cmd()
        # nobuffer + low_delay are important for live streaming
        joined = " ".join(cmd)
        assert "nobuffer" in joined
        assert "low_delay" in joined


# ---------------------------------------------------------------------------
# build_single_frame_cmd
# ---------------------------------------------------------------------------

class TestBuildSingleFrameCmd:
    def test_returns_list_of_strings(self):
        cmd = build_single_frame_cmd()
        assert isinstance(cmd, list)
        assert all(isinstance(s, str) for s in cmd)

    def test_vframes_1(self):
        cmd = build_single_frame_cmd()
        assert "-vframes" in cmd
        idx = cmd.index("-vframes")
        assert cmd[idx + 1] == "1"

    def test_output_is_file_not_pipe(self):
        """Output must be a file path (not pipe:1) so image2 muxer can seek."""
        cmd = build_single_frame_cmd()
        # Must NOT contain pipe:1
        assert "pipe:1" not in cmd
        # Last arg should be a file path ending in .jpg
        assert cmd[-1].endswith(".jpg")

    def test_update_flag_present(self):
        """'-update 1' must be present so ffmpeg doesn't require a pattern."""
        cmd = build_single_frame_cmd()
        assert "-update" in cmd

    def test_timeout_embedded_in_url(self):
        cmd = build_single_frame_cmd(timeout_s=5.0)
        url_arg = next((a for a in cmd if a.startswith("udp://")), None)
        assert url_arg is not None
        assert "timeout=" in url_arg

    def test_custom_out_path(self):
        cmd = build_single_frame_cmd(out_path="/tmp/custom_test.jpg")
        assert cmd[-1] == "/tmp/custom_test.jpg"


# ---------------------------------------------------------------------------
# grab_single_jpeg — offline (no UDP source) behaviour
# ---------------------------------------------------------------------------

class TestGrabSingleJpegOffline:
    """These tests confirm that grab_single_jpeg returns None gracefully when
    there is no UDP source, without hanging for more than a few seconds."""

    def test_returns_none_when_no_source(self):
        # Use an unlikely port to ensure no real source exists
        result = grab_single_jpeg(host="127.0.0.1", port=59876, timeout_s=3.0)
        assert result is None

    def test_does_not_raise(self):
        # Must not raise; just return None
        try:
            grab_single_jpeg(host="127.0.0.1", port=59877, timeout_s=3.0)
        except Exception as exc:
            pytest.fail(f"grab_single_jpeg raised unexpectedly: {exc}")
