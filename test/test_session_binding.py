from klippy import structured_log


def test_session_id_format_round_trip():
    sid = structured_log.make_session_id()
    structured_log.bind_session(sid)
    assert structured_log.get_session() == sid
    assert sid.startswith("k-")


def test_events_dir_derivation():
    from klippy import printer

    assert printer.events_dir_for("/home/pi/printer_data/logs/klippy.log") == (
        "/home/pi/printer_data/logs/events"
    )
    assert printer.events_dir_for(None) is None
