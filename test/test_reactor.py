from klippy import gcode, reactor


class _Printer:
    def is_shutdown(self):
        return False


def test_completion_any_removes_losing_waiter():
    event_loop = reactor.PollReactor()
    interrupt_completion = event_loop.completion()
    interrupt_token = gcode.InterruptToken(_Printer(), interrupt_completion)

    def run_test(eventtime):
        try:
            for result in range(1000):
                drip_completion = event_loop.completion()
                completion = event_loop.completion_any(
                    [drip_completion, interrupt_token]
                )
                event_loop.register_callback(
                    lambda e, c=drip_completion, r=result: c.complete(r)
                )
                assert completion.wait() == result
                event_loop.pause(event_loop.NOW)
                assert not drip_completion.waiting
                assert not interrupt_completion.waiting
            pending_completion = event_loop.completion()
            completion = event_loop.completion_any(
                [pending_completion, interrupt_token]
            )
            assert completion.wait(event_loop.NOW, "timeout") == "timeout"
            event_loop.register_callback(
                lambda e: interrupt_completion.complete(None)
            )
            assert completion.wait() is True
            event_loop.pause(event_loop.NOW)
            assert not pending_completion.waiting
            assert not interrupt_completion.waiting
        finally:
            event_loop.end()

    event_loop.register_callback(run_test)
    try:
        event_loop.run()
        assert len(event_loop._all_greenlets) < 10
    finally:
        event_loop.finalize()
