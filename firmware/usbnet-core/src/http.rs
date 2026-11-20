//! One test page over HTTP/1.1, one request per connection.
//!
//! `GET /` (or `HEAD /`) returns a small HTML page with what the board has
//! seen from the phone: the NCM requests and the DHCP exchange. Other paths
//! get 404, other methods 405. The connection is closed after each answer.

use core::fmt::{self, Write};

use crate::ncm::Seen;
use crate::net::{BOARD_IP, PHONE_IP};

pub const PORT: u16 = 80;
/// Requests longer than this (headers included) get 431.
pub const MAX_REQUEST: usize = 1024;
/// Every response fits in this many bytes.
pub const MAX_RESPONSE: usize = 2048;

/// What the page shows.
#[derive(Clone, Copy, Debug, Default)]
pub struct PageInfo<'a> {
    /// The board's USB serial number (chip ID).
    pub serial: &'a str,
    pub fw: &'a str,
    pub ncm: Seen,
    pub dhcp_offers: u32,
    pub dhcp_acks: u32,
    pub dhcp_naks: u32,
    pub uptime_s: u32,
    /// Pages served before this one.
    pub pages: u32,
}

/// True once `buf` holds the whole request head (it ends with a blank line).
pub fn request_complete(buf: &[u8]) -> bool {
    buf.windows(4).any(|w| w == b"\r\n\r\n")
}

/// A `fmt::Write` over a byte slice. Fails when full.
struct Cursor<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Write for Cursor<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let end = self.len + s.len();
        if end > self.buf.len() {
            return Err(fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

struct Ip([u8; 4]);

impl fmt::Display for Ip {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d] = self.0;
        write!(f, "{a}.{b}.{c}.{d}")
    }
}

struct Opt<T>(Option<T>);

impl<T: fmt::LowerHex> fmt::Display for Opt<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(v) => write!(f, "0x{v:x}"),
            None => f.write_str("not sent"),
        }
    }
}

/// Only characters safe in HTML text are written; anything else becomes '?'.
struct Text<'a>(&'a str);

impl fmt::Display for Text<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for c in self.0.chars() {
            let safe = c.is_ascii_alphanumeric() || matches!(c, '.' | '-' | '_' | ' ');
            f.write_char(if safe { c } else { '?' })?;
        }
        Ok(())
    }
}

fn page(info: &PageInfo, out: &mut [u8]) -> Result<usize, fmt::Error> {
    let mut w = Cursor { buf: out, len: 0 };
    let s = info.ncm;
    write!(
        w,
        "<!doctype html>\n<html><head><meta charset=\"utf-8\">\
<meta name=\"viewport\" content=\"width=device-width\">\
<title>OASA USB test</title></head><body>\n\
<h1>OASA USB network test</h1>\n\
<p>If you can read this, the phone reached the board over USB.</p>\n\
<p>Test firmware. The radio is off.</p>\n<ul>\n\
<li>Board: RAK4631, serial {}, firmware {}</li>\n\
<li>Board address {}, phone address {}</li>\n\
<li>Up {} s, pages served before this one: {}</li>\n\
<li>DHCP: {} offers, {} acks, {} naks</li>\n\
<li>NCM packet filter: {}</li>\n\
<li>NCM input size: {}, max datagrams {}, read back {} times</li>\n\
<li>NCM requests refused: {}</li>\n</ul>\n</body></html>\n",
        Text(info.serial),
        Text(info.fw),
        Ip(BOARD_IP),
        Ip(PHONE_IP),
        info.uptime_s,
        info.pages,
        info.dhcp_offers,
        info.dhcp_acks,
        info.dhcp_naks,
        Opt(s.packet_filter),
        Opt(s.ntb_in_size),
        s.ntb_in_max_datagrams,
        s.get_input_size,
        s.stalled,
    )?;
    Ok(w.len)
}

/// The status a request gets, and whether it is a HEAD.
fn route(req: &[u8]) -> (u16, bool) {
    if req.len() > MAX_REQUEST {
        return (431, false);
    }
    let line_end = match req.windows(2).position(|w| w == b"\r\n") {
        Some(n) => n,
        None => return (400, false),
    };
    let mut parts = req[..line_end].split(|&b| b == b' ');
    let (Some(method), Some(target), Some(version), None) =
        (parts.next(), parts.next(), parts.next(), parts.next())
    else {
        return (400, false);
    };
    if !version.starts_with(b"HTTP/1.") {
        return (400, false);
    }
    let head = method == b"HEAD";
    if method != b"GET" && !head {
        return (405, false);
    }
    // Drop any query string.
    let path = target.split(|&b| b == b'?').next().unwrap_or(b"");
    if path == b"/" || path == b"/index.html" {
        (200, head)
    } else {
        (404, head)
    }
}

/// Builds the whole response to `req` in `out` (at least [`MAX_RESPONSE`]
/// bytes) and returns its length, or `None` if `out` is too small.
pub fn respond(req: &[u8], info: &PageInfo, out: &mut [u8]) -> Option<usize> {
    let (status, head) = route(req);
    let reason = match status {
        200 => "OK",
        400 => "Bad Request",
        404 => "Not Found",
        405 => "Method Not Allowed",
        _ => "Request Header Fields Too Large",
    };
    let mut body = [0u8; MAX_RESPONSE];
    let body_len = if status == 200 {
        page(info, &mut body).ok()?
    } else {
        let mut w = Cursor {
            buf: &mut body,
            len: 0,
        };
        writeln!(w, "{status} {reason}").ok()?;
        w.len
    };
    let ctype = if status == 200 {
        "text/html; charset=utf-8"
    } else {
        "text/plain"
    };
    let mut w = Cursor { buf: out, len: 0 };
    write!(
        w,
        "HTTP/1.1 {status} {reason}\r\nContent-Type: {ctype}\r\n\
Content-Length: {body_len}\r\nCache-Control: no-store\r\nConnection: close\r\n"
    )
    .ok()?;
    if status == 405 {
        w.write_str("Allow: GET, HEAD\r\n").ok()?;
    }
    w.write_str("\r\n").ok()?;
    if !head {
        // Cursor has no write_bytes; the body is ASCII from the writers above.
        w.write_str(core::str::from_utf8(&body[..body_len]).ok()?)
            .ok()?;
    }
    Some(w.len)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::string::String;

    fn info() -> PageInfo<'static> {
        PageInfo {
            serial: "0123456789ABCDEF",
            fw: "0.1.0",
            ncm: Seen {
                packet_filter: Some(0x0e),
                ntb_in_size: Some(16384),
                ntb_in_max_datagrams: 0,
                get_input_size: 1,
                stalled: 0,
            },
            dhcp_offers: 1,
            dhcp_acks: 1,
            dhcp_naks: 0,
            uptime_s: 42,
            pages: 3,
        }
    }

    fn run(req: &str) -> String {
        let mut out = [0u8; MAX_RESPONSE];
        let n = respond(req.as_bytes(), &info(), &mut out).unwrap();
        String::from_utf8(out[..n].to_vec()).unwrap()
    }

    fn split(resp: &str) -> (&str, &str) {
        resp.split_once("\r\n\r\n").unwrap()
    }

    fn content_length(head: &str) -> usize {
        head.lines()
            .find_map(|l| l.strip_prefix("Content-Length: "))
            .unwrap()
            .parse()
            .unwrap()
    }

    #[test]
    fn get_root_serves_the_page() {
        // As Safari sends it, shortened.
        let r = run("GET / HTTP/1.1\r\nHost: 10.71.44.1\r\nAccept: text/html\r\n\r\n");
        let (head, body) = split(&r);
        assert!(head.starts_with("HTTP/1.1 200 OK\r\n"));
        assert!(head.contains("Connection: close"));
        assert_eq!(content_length(head), body.len());
        assert!(body.contains("OASA USB network test"));
        assert!(body.contains("serial 0123456789ABCDEF"));
        assert!(body.contains("phone address 10.71.44.2"));
        assert!(body.contains("NCM packet filter: 0xe"));
        assert!(body.contains("NCM input size: 0x4000"));
        assert!(body.contains("The radio is off"));
    }

    #[test]
    fn unset_values_say_not_sent() {
        let mut i = info();
        i.ncm = Seen::default();
        let mut out = [0u8; MAX_RESPONSE];
        let n = respond(b"GET / HTTP/1.1\r\n\r\n", &i, &mut out).unwrap();
        let r = core::str::from_utf8(&out[..n]).unwrap();
        assert!(r.contains("NCM packet filter: not sent"));
        assert!(r.contains("NCM input size: not sent"));
    }

    #[test]
    fn head_has_no_body() {
        let r = run("HEAD /index.html?x=1 HTTP/1.0\r\n\r\n");
        let (head, body) = split(&r);
        assert!(head.starts_with("HTTP/1.1 200 OK"));
        assert!(content_length(head) > 0);
        assert_eq!(body, "");
    }

    #[test]
    fn errors() {
        assert!(run("GET /favicon.ico HTTP/1.1\r\n\r\n").starts_with("HTTP/1.1 404 "));
        let r = run("POST / HTTP/1.1\r\n\r\n");
        assert!(r.starts_with("HTTP/1.1 405 "));
        assert!(r.contains("Allow: GET, HEAD\r\n"));
        assert!(run("GET /\r\n\r\n").starts_with("HTTP/1.1 400 "));
        assert!(run("GET / SPDY/3\r\n\r\n").starts_with("HTTP/1.1 400 "));
        assert!(run("no line end").starts_with("HTTP/1.1 400 "));
        let long = std::format!("GET /{} HTTP/1.1\r\n\r\n", "a".repeat(MAX_REQUEST));
        assert!(run(&long).starts_with("HTTP/1.1 431 "));
        for r in ["GET /x HTTP/1.1\r\n\r\n", "PUT / HTTP/1.1\r\n\r\n"] {
            let r = run(r);
            let (head, body) = split(&r);
            assert_eq!(content_length(head), body.len());
        }
    }

    #[test]
    fn serial_is_escaped() {
        let mut i = info();
        i.serial = "<script>";
        let mut out = [0u8; MAX_RESPONSE];
        let n = respond(b"GET / HTTP/1.1\r\n\r\n", &i, &mut out).unwrap();
        let r = core::str::from_utf8(&out[..n]).unwrap();
        assert!(!r.contains("<script>"));
        assert!(r.contains("?script?"));
    }

    #[test]
    fn largest_page_fits() {
        let mut i = info();
        i.serial = "FFFFFFFFFFFFFFFF";
        i.fw = "255.255.255";
        i.dhcp_offers = u32::MAX;
        i.dhcp_acks = u32::MAX;
        i.dhcp_naks = u32::MAX;
        i.uptime_s = u32::MAX;
        i.pages = u32::MAX;
        i.ncm = Seen {
            packet_filter: Some(u16::MAX),
            ntb_in_size: Some(u32::MAX),
            ntb_in_max_datagrams: u16::MAX,
            get_input_size: u16::MAX,
            stalled: u16::MAX,
        };
        let mut out = [0u8; MAX_RESPONSE];
        assert!(respond(b"GET / HTTP/1.1\r\n\r\n", &i, &mut out).is_some());
        let mut small = [0u8; 64];
        assert_eq!(respond(b"GET / HTTP/1.1\r\n\r\n", &i, &mut small), None);
    }

    #[test]
    fn completeness() {
        assert!(!request_complete(b"GET / HTTP/1.1\r\nHost: x\r\n"));
        assert!(request_complete(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n"));
    }
}
