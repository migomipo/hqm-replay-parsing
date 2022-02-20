use nalgebra::{Matrix3, Vector3};
use std::cmp::min;

const UXP: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
const UXN: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
const UYP: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);
const UYN: Vector3<f32> = Vector3::new(0.0, -1.0, 0.0);
const UZP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const UZN: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);

const TABLE: [[&'static Vector3<f32>; 3]; 8] = [
    [&UYP, &UXP, &UZP],
    [&UYP, &UZP, &UXN],
    [&UYP, &UZN, &UXP],
    [&UYP, &UXN, &UZN],
    [&UZP, &UXP, &UYN],
    [&UXN, &UZP, &UYN],
    [&UXP, &UZN, &UYN],
    [&UZN, &UXN, &UYN],
];


#[allow(dead_code)]
pub fn convert_matrix_from_network(b: u8, v1: u32, v2: u32) -> Matrix3<f32> {
    let r1 = convert_rot_column_from_network(b, v1);
    let r2 = convert_rot_column_from_network(b, v2);
    let r0 = r1.cross(&r2);
    Matrix3::from_columns(&[r0, r1, r2])
}

#[allow(dead_code)]
fn convert_rot_column_from_network(b: u8, v: u32) -> Vector3<f32> {
    let start = v & 7;

    let mut temp1 = TABLE[start as usize][0].clone();
    let mut temp2 = TABLE[start as usize][1].clone();
    let mut temp3 = TABLE[start as usize][2].clone();
    let mut pos = 3;
    while pos < b {
        let step = (v >> pos) & 3;
        let c1 = (temp1 + temp2).normalize();
        let c2 = (temp2 + temp3).normalize();
        let c3 = (temp1 + temp3).normalize();
        match step {
            0 => {
                temp2 = c1;
                temp3 = c3;
            }
            1 => {
                temp1 = c1;
                temp3 = c2;
            }
            2 => {
                temp1 = c3;
                temp2 = c2;
            }
            3 => {
                temp1 = c1;
                temp2 = c2;
                temp3 = c3;
            }
            _ => panic!(),
        }

        pos += 2;
    }
    (temp1 + temp2 + temp3).normalize()
}

pub struct HQMMessageReader<'a> {
    buf: &'a [u8],
    pub(crate) pos: usize,
    pub(crate) bit_pos: u8,
}

impl<'a> HQMMessageReader<'a> {
    #[allow(dead_code)]
    pub fn get_pos(&self) -> usize {
        self.pos
    }

    fn safe_get_byte(&self, pos: usize) -> u8 {
        if pos < self.buf.len() {
            self.buf[pos]
        } else {
            0
        }
    }

    pub fn read_byte_aligned(&mut self) -> u8 {
        self.align();
        let res = self.safe_get_byte(self.pos);
        self.pos = self.pos + 1;
        return res;
    }

    pub fn read_bytes_aligned(&mut self, n: usize) -> Vec<u8> {
        self.align();

        let mut res = Vec::with_capacity(n);
        for i in self.pos..(self.pos + n) {
            res.push(self.safe_get_byte(i))
        }
        self.pos = self.pos + n;
        return res;
    }

    pub fn read_u16_aligned(&mut self) -> u16 {
        self.align();
        let b1 = self.safe_get_byte(self.pos) as u16;
        let b2 = self.safe_get_byte(self.pos + 1) as u16;
        self.pos = self.pos + 2;
        return b1 | b2 << 8;
    }

    pub fn read_u32_aligned(&mut self) -> u32 {
        self.align();
        let b1 = self.safe_get_byte(self.pos) as u32;
        let b2 = self.safe_get_byte(self.pos + 1) as u32;
        let b3 = self.safe_get_byte(self.pos + 2) as u32;
        let b4 = self.safe_get_byte(self.pos + 3) as u32;
        self.pos = self.pos + 4;
        return b1 | b2 << 8 | b3 << 16 | b4 << 24;
    }

    #[allow(dead_code)]
    pub fn read_pos(&mut self, b: u8, old_value: Option<u32>) -> u32 {
        let pos_type = self.read_bits(2);
        match pos_type {
            0 => {
                let diff = self.read_bits_signed(3);
                let old_value = old_value.unwrap() as i32;
                (old_value + diff).max(0) as u32
            }
            1 => {
                let diff = self.read_bits_signed(6);
                let old_value = old_value.unwrap() as i32;
                (old_value + diff).max(0) as u32
            }
            2 => {
                let diff = self.read_bits_signed(12);
                let old_value = old_value.unwrap() as i32;
                (old_value + diff).max(0) as u32
            }
            3 => self.read_bits(b),
            _ => panic!(),
        }
    }

    #[allow(dead_code)]
    pub fn read_bits_signed(&mut self, b: u8) -> i32 {
        let a = self.read_bits(b);

        if a >= 1 << (b - 1) {
            (-1 << b) | (a as i32)
        } else {
            a as i32
        }
    }

    pub fn read_bits(&mut self, b: u8) -> u32 {
        let mut bits_remaining = b;
        let mut res = 0u32;
        let mut p = 0;
        while bits_remaining > 0 {
            let bits_possible_to_write = 8 - self.bit_pos;
            let bits = min(bits_remaining, bits_possible_to_write);
            let mask = !(!0u32 << bits);
            let a = (self.safe_get_byte(self.pos) as u32 >> self.bit_pos) & mask;

            res = res | (a << p);

            if bits_remaining >= bits_possible_to_write {
                bits_remaining -= bits_possible_to_write;
                self.bit_pos = 0;
                self.pos += 1;
                p += bits;
            } else {
                self.bit_pos += bits_remaining;
                bits_remaining = 0;
            }
        }
        return res;
    }

    pub fn align(&mut self) {
        if self.bit_pos > 0 {
            self.bit_pos = 0;
            self.pos += 1;
        }
    }

    #[allow(dead_code)]
    pub fn next(&mut self) {
        self.pos += 1;
        self.bit_pos = 0;
    }

    pub fn new(buf: &'a [u8]) -> Self {
        HQMMessageReader {
            buf,
            pos: 0,
            bit_pos: 0,
        }
    }
}

#[derive(Debug)]
pub enum HQMObjectPacket {
    None,
    Puck(HQMPuckPacket),
    Skater(HQMSkaterPacket)
}

#[derive(Debug)]
pub struct HQMSkaterPacket {
    pub pos: (u32, u32, u32),
    pub rot: (u32, u32),
    pub stick_pos: (u32, u32, u32),
    pub stick_rot: (u32, u32),
    pub head_rot: u32,
    pub body_rot: u32,
}

#[derive(Debug)]
pub struct HQMPuckPacket {
    pub pos: (u32, u32, u32),
    pub rot: (u32, u32),
}