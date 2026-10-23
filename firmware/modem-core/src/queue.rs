//! A fixed-size FIFO of packets waiting for the radio. No heap.

use crate::frame::MAX_LORA_PAYLOAD;

#[derive(Clone, Copy)]
struct Slot {
    len: u8,
    data: [u8; MAX_LORA_PAYLOAD],
}

/// Holds up to `N` packets of 1 to 255 bytes.
pub struct TxQueue<const N: usize> {
    slots: [Slot; N],
    head: usize,
    len: usize,
}

impl<const N: usize> Default for TxQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> TxQueue<N> {
    pub const fn new() -> Self {
        Self {
            slots: [Slot {
                len: 0,
                data: [0; MAX_LORA_PAYLOAD],
            }; N],
            head: 0,
            len: 0,
        }
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn is_full(&self) -> bool {
        self.len == N
    }

    /// Add a packet at the back. `false` (and nothing stored) if full, empty
    /// or over 255 bytes.
    pub fn push(&mut self, packet: &[u8]) -> bool {
        if self.is_full() || packet.is_empty() || packet.len() > MAX_LORA_PAYLOAD {
            return false;
        }
        let at = (self.head + self.len) % N;
        self.slots[at].len = packet.len() as u8; // <= 255
        self.slots[at].data[..packet.len()].copy_from_slice(packet);
        self.len += 1;
        true
    }

    /// Copy the oldest packet into `out` and remove it. Returns its length.
    pub fn pop_into(&mut self, out: &mut [u8; MAX_LORA_PAYLOAD]) -> Option<usize> {
        if self.len == 0 {
            return None;
        }
        let s = &self.slots[self.head];
        let n = usize::from(s.len);
        out[..n].copy_from_slice(&s.data[..n]);
        self.head = (self.head + 1) % N;
        self.len -= 1;
        Some(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pop(q: &mut TxQueue<3>) -> Option<std::vec::Vec<u8>> {
        let mut b = [0u8; MAX_LORA_PAYLOAD];
        q.pop_into(&mut b).map(|n| b[..n].to_vec())
    }

    #[test]
    fn fifo_order_and_full() {
        let mut q = TxQueue::<3>::new();
        assert!(q.is_empty());
        assert!(q.push(&[1]) && q.push(&[2, 2]) && q.push(&[3; 255]));
        assert!(q.is_full());
        assert!(!q.push(&[4]), "full queue refuses");
        assert_eq!(pop(&mut q), Some(std::vec![1]));
        assert!(q.push(&[5]));
        assert_eq!(pop(&mut q), Some(std::vec![2, 2]));
        assert_eq!(pop(&mut q), Some(std::vec![3; 255]));
        assert_eq!(pop(&mut q), Some(std::vec![5]));
        assert_eq!(pop(&mut q), None);
        assert_eq!(q.len(), 0);
    }

    #[test]
    fn refuses_empty_and_oversize() {
        let mut q = TxQueue::<3>::new();
        assert!(!q.push(&[]));
        assert!(!q.push(&[0; 256]));
        assert!(q.is_empty());
    }

    #[test]
    fn wraps_around_many_times() {
        let mut q = TxQueue::<3>::new();
        for i in 0..100u8 {
            assert!(q.push(&[i, i]));
            assert_eq!(pop(&mut q), Some(std::vec![i, i]));
        }
    }
}
