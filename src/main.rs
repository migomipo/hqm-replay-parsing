mod hqm_parse;

use crate::hqm_parse::{HQMMessageReader, HQMObjectPacket, HQMPuckPacket, HQMSkaterPacket};
use nalgebra::{Matrix3, Point3};
use std::collections::HashMap;
use std::error::Error;

#[derive(Debug, Clone)]
pub struct HQMServerPlayer {
    pub name: String,
    pub team_and_skater: Option<(usize, HQMTeam)>,
}

#[derive(Debug, Clone)]
pub(crate) enum HQMGameObject {
    None,
    Player(HQMSkater),
    Puck(HQMPuck),
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum HQMTeam {
    Red,
    Blue,
}

#[derive(Debug, Clone)]
pub struct HQMSkater {
    pub pos: Point3<f32>,
    pub rot: Matrix3<f32>,
    pub stick_pos: Point3<f32>,  // Measured in meters
    pub stick_rot: Matrix3<f32>, // Rotation matrix
    pub head_rot: f32,           // Radians
    pub body_rot: f32,           // Radians
}

#[derive(Debug, Clone)]
pub struct HQMPuck {
    pub pos: Point3<f32>,
    pub rot: Matrix3<f32>,
}

#[derive(Debug, Clone)]
pub enum HQMMessage {
    PlayerUpdate {
        player_name: String,
        object: Option<(usize, HQMTeam)>,
        player_index: usize,
        in_server: bool,
    },
    Goal {
        team: HQMTeam,
        goal_player_index: Option<usize>,
        assist_player_index: Option<usize>,
    },
    Chat {
        player_index: Option<usize>,
        message: String,
    },
}

#[derive(Debug, Clone)]
struct HQMGameState {
    packet_number: u32,
    red_score: u32,
    blue_score: u32,
    period: u32,
    game_over: bool,
    time: u32,
    goal_message_timer: u32,
    objects: Vec<HQMGameObject>,
    player_list: Vec<Option<HQMServerPlayer>>,
    messages_in_this_packet: Vec<HQMMessage>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().collect();

    let file_name = args[1].as_str();

    let data = std::fs::read(file_name)?;
    let data_len = data.len();
    let mut reader = HQMMessageReader::new(data.as_slice());
    let _ = reader.read_u32_aligned();
    let _bytes = reader.read_u32_aligned() as usize;

    let mut old_saved_packets = HashMap::new();
    // You probably don't need to save all packets,
    // just the most recent 64 or so. Nonetheless, it is easier to just keep all of them for now
    // The only issue will be more RAM usage than necessary

    let mut history = vec![];
    let mut current_player_list = {
        let mut players = vec![];
        for _ in 0..63 {
            players.push(None)
        }
        players
    };
    let mut current_msg_pos = 0;
    while reader.pos < data_len {
        reader.read_byte_aligned(); // Should be 5, but we're not checking
        let game_over = reader.read_bits(1) == 1;
        let red_score = reader.read_bits(8);
        let blue_score = reader.read_bits(8);
        let time = reader.read_bits(16);
        let goal_message_timer = reader.read_bits(16);
        let period = reader.read_bits(8);

        println!(
            "Period {} Time: {}, {}-{}",
            period, time, red_score, blue_score
        );
        let (objects, packet_number) = read_objects(&mut reader, &mut old_saved_packets);

        let message_num = reader.read_bits(16);
        let msg_pos = reader.read_bits(16);
        let mut messages_in_this_packet = vec![];
        for i in 0..message_num {
            let msg_pos_of_this_message = msg_pos + i;
            let msg = read_message(&mut reader);

            if msg_pos_of_this_message >= current_msg_pos {
                match msg {
                    HQMMessage::PlayerUpdate {
                        ref player_name,
                        object,
                        player_index,
                        in_server,
                    } => {
                        if in_server {
                            current_player_list[player_index] = Some(HQMServerPlayer {
                                name: player_name.clone(),
                                team_and_skater: object,
                            })
                        } else {
                            current_player_list[player_index] = None;
                        }
                    }
                    HQMMessage::Goal {
                        team,
                        goal_player_index,
                        assist_player_index,
                    } => {
                        let goal_name = goal_player_index.and_then(|i| {
                            let p = current_player_list[i].as_ref();
                            p.map(|p| p.name.clone())
                        });
                        let assist_name = assist_player_index.and_then(|i| {
                            let p = current_player_list[i].as_ref();
                            p.map(|p| p.name.clone())
                        });
                        println!("Goal for {:?}, {:?}, {:?}", team, goal_name, assist_name);
                    }
                    HQMMessage::Chat {
                        player_index, ref message
                    } => {
                        let name = player_index.and_then(|i| {
                            let p = current_player_list[i].as_ref();
                            p.map(|p| p.name.clone())
                        });
                        if let Some(name) = name {
                            println!("{}: {}", name, message);
                        } else {
                            println!("[Server]: {}", message);
                        }
                    }
                }

                messages_in_this_packet.push(msg);
            }
        }
        current_msg_pos = msg_pos + message_num;

        let state = HQMGameState {
            packet_number,
            red_score,
            blue_score,
            period,
            game_over,
            time,
            goal_message_timer,
            objects,
            player_list: current_player_list.clone(),
            messages_in_this_packet,
        };

        reader.next();

        history.push(state);
    }

    Ok(())
}

fn read_message(reader: &mut HQMMessageReader) -> HQMMessage {
    let message_type = reader.read_bits(6);
    if message_type == 0 {
        // Player update
        let player_index = reader.read_bits(6) as usize;
        let in_server = reader.read_bits(1) == 1;
        let team = match reader.read_bits(2) {
            0 => Some(HQMTeam::Red),
            1 => Some(HQMTeam::Blue),
            _ => None,
        };
        let object_index = match reader.read_bits(6) {
            0x3F => None,
            x => Some(x as usize),
        };
        let object = object_index.zip(team);
        let mut bytes = vec![];
        for _ in 0..31 {
            bytes.push(reader.read_bits(7) as u8);
        }
        if let Ok(s) = String::from_utf8(bytes) {
            let s = s.trim_matches(char::from(0)).to_string();
            HQMMessage::PlayerUpdate {
                player_name: s,
                object,
                player_index,
                in_server,
            }
        } else {
            panic!();
        }
    } else if message_type == 1 {
        // Goal
        let team = match reader.read_bits(2) {
            0 => HQMTeam::Red,
            _ => HQMTeam::Blue,
        };
        let goal_player_index = match reader.read_bits(6) {
            0x3F => None,
            x => Some(x as usize),
        };
        let assist_player_index = match reader.read_bits(6) {
            0x3F => None,
            x => Some(x as usize),
        };
        HQMMessage::Goal {
            team,
            goal_player_index,
            assist_player_index,
        }
    } else if message_type == 2 {
        let player_index = match reader.read_bits(6) {
            0x3F => None,
            x => Some(x as usize),
        };
        let size = reader.read_bits(6);
        let mut bytes = vec![];
        for _ in 0..size {
            bytes.push(reader.read_bits(7) as u8);
        }
        if let Ok(s) = String::from_utf8(bytes) {
            let s = s.trim_matches(char::from(0)).to_string();
            HQMMessage::Chat {
                player_index,
                message: s,
            }
        } else {
            panic!();
        }
    } else {
        panic!("Unknown message type")
    }
}

fn read_objects(
    reader: &mut HQMMessageReader,
    history: &mut HashMap<u32, Vec<HQMObjectPacket>>,
) -> (Vec<HQMGameObject>, u32) {
    let current_packet_num = reader.read_u32_aligned();
    let previous_packet_num = reader.read_u32_aligned();

    let find_old: Option<&[HQMObjectPacket]> =
        history.get(&previous_packet_num).map(|x| x.as_slice());

    let mut packets = vec![];

    for i in 0..32 {
        let is_object = reader.read_bits(1) == 1;
        let packet = if is_object {
            let old_object_in_this_slot = find_old.map(|x| &x[i]);
            let object_type = reader.read_bits(2);
            if object_type == 0 {
                let old_skater = match &old_object_in_this_slot {
                    Some(HQMObjectPacket::Skater(skater)) => Some(skater),
                    _ => None,
                };
                let old_pos = old_skater.map(|x| x.pos);
                let old_rot = old_skater.map(|x| x.rot);

                let x = reader.read_pos(17, old_pos.map(|x| x.0));
                let y = reader.read_pos(17, old_pos.map(|x| x.1));
                let z = reader.read_pos(17, old_pos.map(|x| x.2));
                let r1 = reader.read_pos(31, old_rot.map(|x| x.0));
                let r2 = reader.read_pos(31, old_rot.map(|x| x.1));

                let stick_x = reader.read_pos(13, old_skater.map(|x| x.stick_pos.0));
                let stick_y = reader.read_pos(13, old_skater.map(|x| x.stick_pos.1));
                let stick_z = reader.read_pos(13, old_skater.map(|x| x.stick_pos.2));

                let stick_r1 = reader.read_pos(25, old_skater.map(|x| x.stick_rot.0));
                let stick_r2 = reader.read_pos(25, old_skater.map(|x| x.stick_rot.1));

                let head_rot = reader.read_pos(16, old_skater.map(|x| x.head_rot));
                let body_rot = reader.read_pos(16, old_skater.map(|x| x.body_rot));

                HQMObjectPacket::Skater(HQMSkaterPacket {
                    pos: (x, y, z),
                    rot: (r1, r2),
                    stick_pos: (stick_x, stick_y, stick_z),
                    stick_rot: (stick_r1, stick_r2),
                    head_rot,
                    body_rot,
                })
                // Player
            } else if object_type == 1 {
                // Puck
                let old_puck = match &old_object_in_this_slot {
                    Some(HQMObjectPacket::Puck(puck)) => Some(puck),
                    _ => None,
                };

                let old_pos = old_puck.map(|x| x.pos);
                let old_rot = old_puck.map(|x| x.rot);

                let x = reader.read_pos(17, old_pos.map(|x| x.0));
                let y = reader.read_pos(17, old_pos.map(|x| x.1));
                let z = reader.read_pos(17, old_pos.map(|x| x.2));
                let r1 = reader.read_pos(31, old_rot.map(|x| x.0));
                let r2 = reader.read_pos(31, old_rot.map(|x| x.1));

                HQMObjectPacket::Puck(HQMPuckPacket {
                    pos: (x, y, z),
                    rot: (r1, r2),
                })
            } else {
                panic!("Unknown object type")
            }
        } else {
            HQMObjectPacket::None
        };
        packets.push(packet);
    }

    let objects = packets
        .iter()
        .map(|x| match x {
            HQMObjectPacket::None => HQMGameObject::None,
            HQMObjectPacket::Puck(packet) => {
                let pos = Point3::new(
                    packet.pos.0 as f32 / 1024.0,
                    packet.pos.1 as f32 / 1024.0,
                    packet.pos.2 as f32 / 1024.0,
                );
                let rot = hqm_parse::convert_matrix_from_network(31, packet.rot.0, packet.rot.1);

                HQMGameObject::Puck(HQMPuck { pos, rot })
            }
            HQMObjectPacket::Skater(packet) => {
                let pos = Point3::new(
                    packet.pos.0 as f32 / 1024.0,
                    packet.pos.1 as f32 / 1024.0,
                    packet.pos.2 as f32 / 1024.0,
                );
                let rot = hqm_parse::convert_matrix_from_network(31, packet.rot.0, packet.rot.1);
                let stick_pos = Point3::new(
                    (packet.stick_pos.0 as f32 / 1024.0) + pos.x - 4.0,
                    (packet.stick_pos.1 as f32 / 1024.0) + pos.y - 4.0,
                    (packet.stick_pos.2 as f32 / 1024.0) + pos.z - 4.0,
                );
                let stick_rot = hqm_parse::convert_matrix_from_network(
                    25,
                    packet.stick_rot.0,
                    packet.stick_rot.1,
                );
                HQMGameObject::Player(HQMSkater {
                    pos,
                    rot,
                    stick_pos,
                    stick_rot,
                    head_rot: (packet.head_rot as f32 - 16384.0) / 8192.0,
                    body_rot: (packet.body_rot as f32 - 16384.0) / 8192.0,
                })
            }
        })
        .collect();

    history.insert(current_packet_num, packets);
    (objects, current_packet_num)
}
