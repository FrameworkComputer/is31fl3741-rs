use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};

#[derive(Default)]
pub struct MockI2c {
    pub writes: Vec<Vec<u8>>,
    pub read_value: u8,
    pub fail: bool,
    pub fail_at: Option<usize>,
    pub transactions: usize,
    pub expected_address: Option<u8>,
}

impl ErrorType for MockI2c {
    type Error = ErrorKind;
}

impl I2c for MockI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        assert_eq!(address, self.expected_address.unwrap_or(0x30));
        let index = self.transactions;
        self.transactions += 1;
        if self.fail || self.fail_at == Some(index) {
            return Err(ErrorKind::Other);
        }
        for operation in operations {
            match operation {
                Operation::Write(bytes) => self.writes.push(bytes.to_vec()),
                Operation::Read(bytes) => bytes.fill(self.read_value),
            }
        }
        Ok(())
    }
}
