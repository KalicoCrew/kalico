use super::*;

fn tmp(name: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!("kalico-jsonl-test-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&p);
    std::fs::create_dir_all(&p).unwrap();
    p.push("host-rust.jsonl");
    p
}

#[test]
fn writes_lines_to_base_file() {
    let path = tmp("basic");
    let mut w = RotatingJsonlWriter::new(&path, 1024, 3, FSYNC_INTERVAL).unwrap();
    w.write_all(b"{\"a\":1}\n").unwrap();
    w.flush().unwrap();
    let contents = std::fs::read_to_string(&path).unwrap();
    assert_eq!(contents, "{\"a\":1}\n");
}

#[test]
fn rotates_when_exceeding_max_bytes() {
    let path = tmp("rotate");
    let mut w = RotatingJsonlWriter::new(&path, 8, 3, FSYNC_INTERVAL).unwrap();
    w.write_all(b"AAAAAAA\n").unwrap();
    w.write_all(b"BBBBBBB\n").unwrap();
    w.flush().unwrap();
    let base = std::fs::read_to_string(&path).unwrap();
    let mut rotated_name = path.as_os_str().to_os_string();
    rotated_name.push(".1");
    let rotated = std::fs::read_to_string(PathBuf::from(&rotated_name)).unwrap();
    assert_eq!(base, "BBBBBBB\n");
    assert_eq!(rotated, "AAAAAAA\n");
}

#[test]
fn drops_oldest_beyond_backup_count() {
    let path = tmp("cascade");
    let mut w = RotatingJsonlWriter::new(&path, 4, 2, FSYNC_INTERVAL).unwrap();
    for i in 0..5u8 {
        w.write_all(&[b'0' + i, b'\n', b'x', b'\n']).unwrap();
    }
    w.flush().unwrap();
    assert!(path.exists());
    let mut p1 = path.as_os_str().to_os_string();
    p1.push(".1");
    assert!(PathBuf::from(&p1).exists());
    let mut p2 = path.as_os_str().to_os_string();
    p2.push(".2");
    assert!(PathBuf::from(&p2).exists());
    let mut p3 = path.as_os_str().to_os_string();
    p3.push(".3");
    assert!(!PathBuf::from(&p3).exists());
}
