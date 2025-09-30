use proc_macro::TokenStream;
use syn::{parse_macro_input, parse_quote, visit_mut::{self, VisitMut}, Expr, ItemFn};
use quote::{quote, ToTokens};

struct AwaitTime;

impl VisitMut for AwaitTime {
    fn visit_expr_mut(&mut self, node: &mut syn::Expr) {
        if let Expr::Await(expr) = &node {
            // (async || {defmt::info!(">> Awaiting"); let _r = #expr; defmt::info!(">> Back"); _r})().await
            *node = parse_quote! {
                (async || {let _s = embassy_time::Instant::now(); let _r = #expr; _await_elapsed += embassy_time::Instant::now() - _s; _r})().await
            };
            return;
        }
        visit_mut::visit_expr_mut(self, node);
    }
}


/// Setup function for sync/async profiling (runtime)
///
/// Sets up some local variables for tracking sync/async execution time. Wraps all first level
/// awaits to determine how long the await takes ands accumulates the total await time. It does
/// this by wrapping the async call in a closure and running the closure.
///
/// This doesn't fully measure how long a task actually awaits for (like if there is sync code in
/// an async function), but it should be good enough for optimizing sync code. Using embassy's
/// trace would probably provide a more accurate measurement.
///
/// Example Initial:
/// async fn example_gen(...) -> ! {
///     ...
///     func().await;
///     ...
///     match fut.await {
///         ...
///     }
///     ...
///     async {
///         func().await;
///     }.await;
///     ...
/// }
///
/// Example Generated:
/// async fn example_gen(...) -> ! {
///     let mut _await_elapsed = embassy_time::Duration::default(); // Generated
///     let mut _start_time = embassy_time::Instant::now();         // Generated
///     ...
///     (async || {
///         let _s = embassy_time::Instant::now();
///         let _r = func().await;
///         _await_elapsed += embassy_time::Instant::now() - _s;
///         _r
///     })().await;
///     ...
///     match (async || {let _s = embassy_time::Instant::now(); let _r = fut.await; _await_elapsed += embassy_time::Instant::now() - _s; _r})().await {
///         ...
///     }
///     ...
///     (async || {
///         let _s = embassy_time::Instant::now();
///         let _r = async {
///             func().await;
///         }.await;
///         _await_elapsed += embassy_time::Instant::now() - _s;
///         _r
///     })().await;
/// }
#[proc_macro_attribute]
pub fn profile(_args: TokenStream, input: TokenStream) -> TokenStream {
    let mut func = parse_macro_input!(input as ItemFn);
    let lines = &mut func.block.stmts;

    let print = syn::parse(quote! {
        let mut _start_time = embassy_time::Instant::now();
    }.into()).unwrap();
    lines.insert(0, print);
    let print = syn::parse(quote! {
        let mut _await_elapsed = embassy_time::Duration::default();
    }.into()).unwrap();
    lines.insert(0, print);

    AwaitTime.visit_item_fn_mut(&mut func);

    func.into_token_stream().into()
}

/// Set this point as the starting point of the profiling
#[proc_macro]
pub fn set(_args: TokenStream) -> TokenStream {
    quote! {
        _await_elapsed = embassy_time::Duration::default();
        _start_time = embassy_time::Instant::now();
    }.into()
}

/// Prints time spent in sync and async code at this point
#[proc_macro]
pub fn print(_args: TokenStream) -> TokenStream {
    quote! {
        let _e = embassy_time::Instant::now() - _start_time;
        defmt::info!("Sync Time: {}uS, Async Time: {}uS", (_e - _await_elapsed).as_micros(), _await_elapsed.as_micros());
    }.into()
}

/// Get time spent in sync code at this point
#[proc_macro]
pub fn get_sync(_args: TokenStream) -> TokenStream {
    quote! {
        (embassy_time::Instant::now() - _start_time - _await_elapsed).as_micros()
    }.into()
}

/// Get time spent in async code at this point
#[proc_macro]
pub fn get_async(_args: TokenStream) -> TokenStream {
    quote! {
        _await_elapsed.as_micros()
    }.into()
}
