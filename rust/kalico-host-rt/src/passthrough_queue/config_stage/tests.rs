use super::*;

#[test]
fn config_cmds_drain_before_init_cmds() {
    let mut cs = ConfigStage::new();
    cs.add_config_cmd(vec![0x01]);
    cs.add_config_cmd(vec![0x02]);
    cs.add_init_cmd(vec![0x0A]);
    cs.add_init_cmd(vec![0x0B]);

    cs.begin_config_send();

    assert_eq!(cs.next_config_entry(), Some(vec![0x01]));
    assert_eq!(cs.next_config_entry(), Some(vec![0x02]));
    assert_eq!(cs.next_config_entry(), Some(vec![0x0A]));
    assert_eq!(cs.next_config_entry(), Some(vec![0x0B]));
    assert_eq!(cs.next_config_entry(), None);
    assert_eq!(cs.phase(), ConfigStagePhase::Runtime);
}

#[test]
fn begin_config_send_transitions_correctly() {
    let mut cs = ConfigStage::new();
    assert_eq!(cs.phase(), ConfigStagePhase::Collecting);

    cs.begin_config_send();
    assert_eq!(cs.phase(), ConfigStagePhase::SendingConfig);

    assert_eq!(cs.next_config_entry(), None);
    assert_eq!(cs.phase(), ConfigStagePhase::Runtime);
}

#[test]
fn cannot_add_commands_after_begin_config_send() {
    let mut cs = ConfigStage::new();
    assert!(cs.add_config_cmd(vec![0x01]));
    assert!(cs.add_init_cmd(vec![0x02]));
    assert!(cs.add_restart_cmd(vec![0x03]));

    cs.begin_config_send();

    assert!(!cs.add_config_cmd(vec![0x04]));
    assert!(!cs.add_init_cmd(vec![0x05]));
    assert!(!cs.add_restart_cmd(vec![0x06]));
}

#[test]
fn restart_cmds_are_stored_and_retrievable() {
    let mut cs = ConfigStage::new();
    cs.add_restart_cmd(vec![0xAA]);
    cs.add_restart_cmd(vec![0xBB]);

    assert_eq!(cs.restart_cmds().len(), 2);
    assert_eq!(cs.restart_cmds()[0], vec![0xAA]);
    assert_eq!(cs.restart_cmds()[1], vec![0xBB]);
}

#[test]
fn next_config_entry_returns_none_during_collecting() {
    let mut cs = ConfigStage::new();
    cs.add_config_cmd(vec![0x01]);
    assert_eq!(cs.next_config_entry(), None);
}
