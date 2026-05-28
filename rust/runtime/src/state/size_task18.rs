use super::RuntimeContext;

#[test]
#[allow(clippy::integer_division)]
fn print_runtime_context_size() {
    let size = core::mem::size_of::<RuntimeContext>();
    eprintln!(
        "[Task 18] size_of::<RuntimeContext>() = {} bytes (={} KB)",
        size,
        size / 1024
    );
}
