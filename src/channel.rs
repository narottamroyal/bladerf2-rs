//! RF channel and direction types.

/// RF channel identifier (the bladeRF 2.0 micro has two RX and two TX).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    /// RX channel 0.
    Rx0,
    /// TX channel 0.
    Tx0,
    /// RX channel 1.
    Rx1,
    /// TX channel 1.
    Tx1,
}

impl Channel {
    /// Direction of this channel.
    pub fn direction(self) -> Direction {
        match self {
            Self::Rx0 | Self::Rx1 => Direction::Rx,
            Self::Tx0 | Self::Tx1 => Direction::Tx,
        }
    }

    /// Channel index within its direction (0 or 1).
    pub fn index(self) -> usize {
        match self {
            Self::Rx0 | Self::Tx0 => 0,
            Self::Rx1 | Self::Tx1 => 1,
        }
    }
}

impl From<Channel> for u16 {
    fn from(value: Channel) -> Self {
        match value {
            Channel::Rx0 => 0,
            Channel::Tx0 => 1,
            Channel::Rx1 => 2,
            Channel::Tx1 => 3,
        }
    }
}

/// RF direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Receive.
    Rx,
    /// Transmit.
    Tx,
}

impl From<Direction> for u16 {
    fn from(value: Direction) -> Self {
        match value {
            Direction::Rx => 0,
            Direction::Tx => 1,
        }
    }
}
