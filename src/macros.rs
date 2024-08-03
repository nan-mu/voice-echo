// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
#[macro_export]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

///
#[macro_export]
macro_rules! mk_static_ref {
    ($val:expr) => {{
        use alloc::boxed::Box;
        let val = Box::new($val);
        let val: &'static _ = Box::leak(val);
        val
    }};
}
