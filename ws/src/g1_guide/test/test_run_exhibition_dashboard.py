"""Process ownership checks for the one-command exhibition session."""

from unittest.mock import Mock, patch

from scripts.run_exhibition_dashboard import _stop_owned


def test_stop_sends_one_sigint_to_owned_wrapper():
    process = Mock(pid=4321, returncode=None)
    process.poll.return_value = None

    def stopped(timeout):
        process.returncode = 0

    process.wait.side_effect = stopped
    with patch("scripts.run_exhibition_dashboard.os.killpg", create=True) as kill, patch(
            "scripts.run_exhibition_dashboard.live_group_members", return_value=[]):
        assert _stop_owned(process, "owned")
    process.send_signal.assert_called_once()
    assert kill.call_count == 0


def test_stop_rejects_lingering_owned_group_members():
    process = Mock(pid=5678, returncode=0)
    process.poll.return_value = 0
    with patch("scripts.run_exhibition_dashboard.os.killpg", create=True) as kill, patch(
            "scripts.run_exhibition_dashboard.live_group_members", return_value=[999]), patch(
            "scripts.run_exhibition_dashboard.time.monotonic", side_effect=[0.0, 6.0]):
        assert not _stop_owned(process, "owned")
    assert kill.call_args.args[0] == 5678
