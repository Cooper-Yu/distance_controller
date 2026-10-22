"""Owned controller stop handoff: pure clocks/processes/graph, no ROS node."""

import math
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as N
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_runtime import PlannedRunner


class Fixture:
    def __init__(self, kill_only=False, never_exit=False, graph_delay=0.03):
        self.now = 1.0
        self.exit_at = math.inf
        self.kill_only, self.never_exit, self.graph_delay = kill_only, never_exit, graph_delay
        self.signals, self.publications, self.events = [], [], []
        self.runner = object.__new__(PlannedRunner)
        r = self.runner
        r.child = Mock(pid=424242)
        self.child = r.child
        self.child.poll.side_effect = lambda: 0 if self.now >= self.exit_at else None
        r.cancel_requested = True
        r.client = Mock()
        r.client.service_is_ready.return_value = False
        r.service = N(Request=lambda: N(action=None))
        r.node, r.ros = Mock(), Mock()
        r.ros.spin_once.side_effect = self.spin
        r.node.get_publishers_info_by_topic.side_effect = self.endpoints
        r.node.create_publisher.side_effect = self.create
        r.node.destroy_publisher.side_effect = lambda _pub: self.events.append('destroy')
        r.twist = lambda: N(linear=N(x=0, y=0), angular=N(z=0))
        r.stop_pub = None

    def endpoints(self, _topic):
        return ['old_endpoint'] if self.now < self.exit_at + self.graph_delay else []

    def spin(self, _node, timeout_sec):
        self.now += max(0.001, timeout_sec)

    def killpg(self, _pid, sig):
        if self.now >= self.exit_at:
            raise ProcessLookupError
        if sig == 0:
            return
        self.signals.append((sig, self.now))
        if not self.never_exit and (not self.kill_only or sig == signal.SIGKILL):
            self.exit_at = self.now + 0.02

    def create(self, *_args):
        assert self.child.poll() is not None
        assert not self.endpoints('/cmd_vel')
        self.events.append('create')
        publisher = Mock()
        publisher.publish.side_effect = lambda _msg: self.publications.append(self.now)
        return publisher

    def run(self):
        with (
            patch('action_runtime.time.monotonic', side_effect=lambda: self.now),
            patch('action_runtime.os.killpg', side_effect=self.killpg),
        ):
            self.runner.stop_owned()


def test_live_child_and_graph_release_precede_parent_zero_publisher():
    f = Fixture(graph_delay=0.30)
    f.run()
    assert f.signals[0][0] == signal.SIGINT
    assert f.events == ['create', 'destroy']
    assert f.publications[0] >= f.exit_at + 0.30 + 0.10 - 1e-9
    assert 19 <= len(f.publications) <= 21
    assert min(b - a for a, b in zip(f.publications, f.publications[1:])) >= 0.05 - 1e-9
    assert f.runner.stop_pub is None and f.runner.child is None
    assert f.runner.cancel_requested is True


def test_latched_cpp_fault_does_not_send_another_cancel_or_rewrite_fault():
    f = Fixture()
    f.runner.owned_fault_observed = True
    f.runner.owned_fault_child = f.child
    f.run()
    assert f.runner.owned_zero_acknowledged is True
    f.runner.client.service_is_ready.assert_not_called()
    f.runner.client.call_async.assert_not_called()


def test_existing_owner_cancel_ack_precedes_process_termination():
    f = Fixture()
    f.runner.client.service_is_ready.return_value = True
    future = Mock()
    future.done.return_value = True
    future.result.return_value = N(accepted=True)
    f.runner.client.call_async.return_value = future
    f.run()
    assert f.runner.client.call_async.call_args.args[0].action == 'cancel'
    assert f.runner.owned_zero_acknowledged is True


def test_unresponsive_service_and_child_escalate_without_five_second_wait():
    f = Fixture(kill_only=True)
    f.runner.client.service_is_ready.return_value = True
    f.runner.client.call_async.return_value.done.return_value = False
    f.run()
    assert [sig for sig, _ in f.signals] == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert f.signals[-1][1] - 1 <= 0.36
    assert f.runner.owned_zero_acknowledged is False
    assert f.now < 3


def test_unreaped_child_never_creates_stop_publisher_and_retains_handle():
    f = Fixture(never_exit=True)
    with pytest.raises(RuntimeError, match='stop unverified'):
        f.run()
    assert f.now < 1.51
    assert f.runner.child is f.child
    f.runner.node.create_publisher.assert_not_called()


def test_foreign_or_persistent_graph_endpoint_never_competes():
    f = Fixture()
    f.runner.node.get_publishers_info_by_topic.side_effect = lambda _topic: ['foreign']
    with pytest.raises(RuntimeError, match='did not release'):
        f.run()
    f.runner.node.create_publisher.assert_not_called()


def test_service_exception_does_not_skip_child_termination():
    f = Fixture()
    f.runner.client.service_is_ready.side_effect = RuntimeError('discovery failure')
    f.run()
    assert f.signals and f.runner.child is None


def test_publisher_creation_failure_occurs_only_after_reaping():
    f = Fixture()
    f.runner.node.create_publisher.side_effect = RuntimeError('creation failure')
    with pytest.raises(RuntimeError, match='creation failure'):
        f.run()
    assert f.child.poll() is not None and f.runner.child is None


@pytest.mark.parametrize('exception', [RuntimeError('publish failed'), KeyboardInterrupt()])
def test_zero_failure_still_destroys_parent_publisher(exception):
    f = Fixture()

    def create(*args):
        pub = f.create(*args)
        pub.publish.side_effect = exception
        return pub

    f.runner.node.create_publisher.side_effect = create
    with pytest.raises(type(exception)):
        f.run()
    assert f.events == ['create', 'destroy']
    assert f.runner.stop_pub is None
    assert f.runner.cancel_requested is True


def test_prepare_preserves_first_fault_when_cleanup_fails():
    r = object.__new__(PlannedRunner)
    first = RuntimeError('START_CORRECTION_HEADING')
    r.start = Mock()
    r.wait_idle = Mock(side_effect=first)
    r.stop_owned = Mock(side_effect=RuntimeError('child not reaped'))
    r.child = object()
    original_child = r.child
    r.cancel_requested = True
    with pytest.raises(RuntimeError, match='START_CORRECTION_HEADING') as caught:
        r.prepare_start()
    assert caught.value is first
    assert first.cleanup_error == 'child not reaped'
    assert r.child is original_child
    assert r.cancel_requested is True


def test_no_owned_child_creates_no_new_publisher():
    f = Fixture()
    f.runner.child = None
    f.run()
    f.runner.node.create_publisher.assert_not_called()
    assert f.runner.cancel_requested is True


def test_prior_child_fault_cannot_acknowledge_current_child_zero():
    f = Fixture()
    f.runner.owned_fault_observed = True
    f.runner.owned_fault_child = object()
    f.run()
    assert f.runner.owned_zero_acknowledged is False
    f.runner.client.service_is_ready.assert_called_once()


def test_start_resets_prior_child_fault_evidence():
    f = Fixture()
    f.runner.owned_fault_observed = True
    f.runner.owned_fault_child = f.child
    with patch('action_runtime.Runner.start') as start:
        f.runner.start(['not-executed'])
    start.assert_called_once_with(['not-executed'])
    assert not f.runner.owned_fault_observed
    assert f.runner.owned_fault_child is None
