/*
 * name:        SX1272
 * description: 860 MHz to 1020 MHz Low Power Long Range Transceiver featuring the LoRa (TM) long range modem
 * manuf:       Semtech
 * version:     Version 0.1
 * url:         https://www.semtech.com/uploads/documents/sx1272.pdf
 * date:        2016-08-01
 * author       https://chisl.io/
 * file:        SX1272_LORA.hpp
 */

/*                                                                                                       *
 *                                   THIS FILE IS AUTOMATICALLY CREATED                                  *
 *                                    D O     N O T     M O D I F Y  !                                   *
 *                                                                                                       */

#include <cinttypes>

/* Derive from class SX1272_LORA_Base and implement the read and write functions! */

/* SX1272: 860 MHz to 1020 MHz Low Power Long Range Transceiver featuring the LoRa (TM) long range modem */
class SX1272_LORA_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	virtual uint16_t read16(uint16_t address, uint16_t n=16) = 0;  // 16 bit read
	virtual void write(uint16_t address, uint16_t value, uint16_t n=16) = 0;  // 16 bit write
	virtual uint32_t read32(uint16_t address, uint16_t n=32) = 0;  // 32 bit read
	virtual void write(uint16_t address, uint32_t value, uint16_t n=32) = 0;  // 32 bit write
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG Fifo                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Fifo:
	 * LoRaTM base-band FIFO data input/output. FIFO is cleared an not accessible when device is in SLEEP mode
	 */
	struct Fifo
	{
		static const uint16_t __address = 0;
		
		/* Bits Fifo: */
		struct Fifo_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register Fifo */
	void setFifo(uint8_t value)
	{
		write(Fifo::__address, value, 8);
	}
	
	/* Get register Fifo */
	uint8_t getFifo()
	{
		return read8(Fifo::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG OpMode                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG OpMode:
	 */
	struct OpMode
	{
		static const uint16_t __address = 1;
		
		/* Bits LongRangeMode: */
		/*
		 * This bit can be modified only in Sleep mode.
		 * A write operation on other device modes is ignored.
		 */
		struct LongRangeMode
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t FSK_OOK_MODE = 0b0; // FSK/OOK Mode
			static const uint8_t LORA_MODE = 0b1; // LoRaTM Mode
		};
		/* Bits AccesSharedReg: */
		/*
		 * This bit operates when device is in Lora mode; if set it allows access to FSK registers page located in
		 * address space (0x0D:0x3F) while in LoRa mode
		 */
		struct AccesSharedReg
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t LORA = 0b0; // Access LoRa registers page 0x0D: 0x3F
			static const uint8_t FSK = 0b1; // Access FSK registers page (in mode LoRa) 0x0D: 0x3F
		};
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b00111000; // [3,4,5]
		};
		/* Bits Mode: */
		/* Device modes  */
		struct Mode
		{
			/* MODE rwt */
			static const uint8_t dflt = 0b001; // 3'b1
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t SLEEP = 0b00; // 
			static const uint8_t STDBY = 0b01; // 
			static const uint8_t FSTX = 0b10; // Frequency synthesis TX
			static const uint8_t TX = 0b11; // Transmit
			static const uint8_t FSRX = 0b100; // Frequency synthesis RX
			static const uint8_t RXCONTINUOUS = 0b101; // Receive continuous
			static const uint8_t RXSINGLE = 0b110; // receive single
			static const uint8_t CAD = 0b111; // Channel activity detection §
		};
	};
	
	/* Set register OpMode */
	void setOpMode(uint8_t value)
	{
		write(OpMode::__address, value, 8);
	}
	
	/* Get register OpMode */
	uint8_t getOpMode()
	{
		return read8(OpMode::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                              REG Fr                                               *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Fr:
	 * RF carrier frequency f = F(XOSC) * Frf / 2^19
	 * Resolution is 61.035 Hz if F(XOSC) = 32 MHz.
	 * Default value is 0xe4c000 = 915 MHz.
	 * Register values must be modified only when device is in SLEEP or STANDBY mode.
	 */
	struct Fr
	{
		static const uint16_t __address = 6;
		
		/* Bits Fr: */
		struct Fr_
		{
			/* MODE rwt */
			static const uint32_t dflt = 0b111001001100000000000000; // 24'he4c000
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register Fr */
	void setFr(uint32_t value)
	{
		write(Fr::__address, value, 24);
	}
	
	/* Get register Fr */
	uint32_t getFr()
	{
		return read32(Fr::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG PaConfig                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PaConfig:
	 */
	struct PaConfig
	{
		static const uint16_t __address = 9;
		
		/* Bits PaSelect: */
		/*
		 * Selects PA output pin
		 * 0 -> RFIO pin. Output power is limited to 13 dBm.
		 * 1 -> PA_BOOST pin. Output power is limited to 20 dBm
		 */
		struct PaSelect
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t mask = 0b01110000; // [4,5,6]
		};
		/* Bits OutputPower: */
		/*
		 * power amplifier max output power:
		 * Pout = 2 + OutputPower(3:0) on PA_BOOST.
		 * Pout = -1 + OutputPower(3:0) on RFIO.
		 */
		struct OutputPower
		{
			/* MODE rw */
			static const uint8_t dflt = 0b1111; // 4'hf
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
		};
	};
	
	/* Set register PaConfig */
	void setPaConfig(uint8_t value)
	{
		write(PaConfig::__address, value, 8);
	}
	
	/* Get register PaConfig */
	uint8_t getPaConfig()
	{
		return read8(PaConfig::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG PaRamp                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PaRamp:
	 */
	struct PaRamp
	{
		static const uint16_t __address = 10;
		
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits LowPnTxPllOff: */
		/*
		 * 1 -> Low consumption PLL is used in receive and transmit mode
		 * 0 -> Low consumption PLL in receive mode, low phase noise PLL in transmit mode.
		 */
		struct LowPnTxPllOff
		{
			/* MODE rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits PaRamp: */
		/* Rise/Fall time of ramp up/down in FSK  */
		struct PaRamp_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b1001; // 4'h9
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
			static const uint8_t PA_RAMP_3_4_ms = 0b000; // 
			static const uint8_t PA_RAMP_2_ms = 0b001; // 
			static const uint8_t PA_RAMP_1_ms = 0b010; // 
			static const uint8_t PA_RAMP_500_us = 0b011; // 
			static const uint8_t PA_RAMP_250_us = 0b100; // 
			static const uint8_t PA_RAMP_125_us = 0b101; // 
			static const uint8_t PA_RAMP_100_us = 0b110; // 
			static const uint8_t PA_RAMP_62_us = 0b111; // 
			static const uint8_t PA_RAMP_50_us = 0b1000; // 
			static const uint8_t PA_RAMP_40_us = 0b1001; // 
			static const uint8_t PA_RAMP_31_us = 0b1010; // 
			static const uint8_t PA_RAMP_25_us = 0b1011; // 
			static const uint8_t PA_RAMP_20_us = 0b1100; // 
			static const uint8_t PA_RAMP_15_us = 0b1101; // 
			static const uint8_t PA_RAMP_12_us = 0b1110; // 
			static const uint8_t PA_RAMP_10_us = 0b1111; // 
		};
	};
	
	/* Set register PaRamp */
	void setPaRamp(uint8_t value)
	{
		write(PaRamp::__address, value, 8);
	}
	
	/* Get register PaRamp */
	uint8_t getPaRamp()
	{
		return read8(PaRamp::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Ocp                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Ocp:
	 */
	struct Ocp
	{
		static const uint16_t __address = 11;
		
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits OcpOn: */
		/*
		 * Enables overload current protection (OCP) for PA:
		 * 0 -> OCP disabled
		 * 1 -> OCP enabled
		 */
		struct OcpOn
		{
			/* MODE rw */
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits OcpTrim: */
		/*
		 * Trimming of OCP current:
		 * Imax =
		 * 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA)
		 * -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240 mA)
		 * 240mA for higher settings
		 * Default Imax = 100mA
		 */
		struct OcpTrim
		{
			/* MODE rw */
			static const uint8_t dflt = 0b01011; // 5'b1011
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register Ocp */
	void setOcp(uint8_t value)
	{
		write(Ocp::__address, value, 8);
	}
	
	/* Get register Ocp */
	uint8_t getOcp()
	{
		return read8(Ocp::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Lna                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Lna:
	 */
	struct Lna
	{
		static const uint16_t __address = 12;
		
		/* Bits LnaGain: */
		/*
		 * LNA gain setting:
		 * 000 -> not used
		 * 111 -> not used
		 */
		struct LnaGain
		{
			/* MODE rwx */
			static const uint8_t dflt = 0b001; // 3'd1
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t G1 = 0b01; // maximum gain
			static const uint8_t G2 = 0b10; // 
			static const uint8_t G3 = 0b11; // 
			static const uint8_t G4 = 0b100; // 
			static const uint8_t G5 = 0b101; // 
			static const uint8_t G6 = 0b110; // minimum gain
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b00011100; // [2,3,4]
		};
		/* Bits LnaBoost: */
		struct LnaBoost
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t DEFAULT = 0b00; // Default LNA current
			static const uint8_t BOOST = 0b11; // Boost on, 150% LNA current.
		};
	};
	
	/* Set register Lna */
	void setLna(uint8_t value)
	{
		write(Lna::__address, value, 8);
	}
	
	/* Get register Lna */
	uint8_t getLna()
	{
		return read8(Lna::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG FifoAddrPtr                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FifoAddrPtr:
	 * SPI interface address pointer in FIFO data buffer.
	 */
	struct FifoAddrPtr
	{
		static const uint16_t __address = 13;
		
		/* Bits FifoAddrPtr: */
		struct FifoAddrPtr_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoAddrPtr */
	void setFifoAddrPtr(uint8_t value)
	{
		write(FifoAddrPtr::__address, value, 8);
	}
	
	/* Get register FifoAddrPtr */
	uint8_t getFifoAddrPtr()
	{
		return read8(FifoAddrPtr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG FifoTxBaseAddr                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FifoTxBaseAddr:
	 * write base address in FIFO data buffer for TX modulator
	 */
	struct FifoTxBaseAddr
	{
		static const uint16_t __address = 14;
		
		/* Bits FifoTxBaseAddr: */
		struct FifoTxBaseAddr_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b10000000; // 8'h80
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoTxBaseAddr */
	void setFifoTxBaseAddr(uint8_t value)
	{
		write(FifoTxBaseAddr::__address, value, 8);
	}
	
	/* Get register FifoTxBaseAddr */
	uint8_t getFifoTxBaseAddr()
	{
		return read8(FifoTxBaseAddr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG FifoRxBaseAddr                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FifoRxBaseAddr:
	 * read base address in FIFO data buffer for RX demodulator
	 */
	struct FifoRxBaseAddr
	{
		static const uint16_t __address = 15;
		
		/* Bits FifoRxBaseAddr: */
		struct FifoRxBaseAddr_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoRxBaseAddr */
	void setFifoRxBaseAddr(uint8_t value)
	{
		write(FifoRxBaseAddr::__address, value, 8);
	}
	
	/* Get register FifoRxBaseAddr */
	uint8_t getFifoRxBaseAddr()
	{
		return read8(FifoRxBaseAddr::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                      REG FifoRxCurrentAddr                                       *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FifoRxCurrentAddr:
	 * Start address (in data buffer) of last packet received
	 */
	struct FifoRxCurrentAddr
	{
		static const uint16_t __address = 16;
		
		/* Bits FifoRxCurrentAddr: */
		struct FifoRxCurrentAddr_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoRxCurrentAddr */
	void setFifoRxCurrentAddr(uint8_t value)
	{
		write(FifoRxCurrentAddr::__address, value, 8);
	}
	
	/* Get register FifoRxCurrentAddr */
	uint8_t getFifoRxCurrentAddr()
	{
		return read8(FifoRxCurrentAddr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG IrqFlagsMask                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG IrqFlagsMask:
	 */
	struct IrqFlagsMask
	{
		static const uint16_t __address = 17;
		
		/* Bits RxTimeoutMask: */
		/* Timeout interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct RxTimeoutMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits RxDoneMask: */
		/* Packet reception complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct RxDoneMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits PayloadCrcErrorMask: */
		/* Payload CRC error interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct PayloadCrcErrorMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits ValidHeaderMask: */
		/* Valid header received in Rx mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct ValidHeaderMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits TxDoneMask: */
		/* FIFO Payload transmission complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct TxDoneMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits CadDoneMask: */
		/* CAD complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct CadDoneMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits FhssChangeChannelMask: */
		/* FHSS change channel interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct FhssChangeChannelMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits CadDetectedMask: */
		/* Cad Detected Interrupt Mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct CadDetectedMask
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register IrqFlagsMask */
	void setIrqFlagsMask(uint8_t value)
	{
		write(IrqFlagsMask::__address, value, 8);
	}
	
	/* Get register IrqFlagsMask */
	uint8_t getIrqFlagsMask()
	{
		return read8(IrqFlagsMask::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG IrqFlags                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG IrqFlags:
	 */
	struct IrqFlags
	{
		static const uint16_t __address = 18;
		
		/* Bits RxTimeout: */
		/* Timeout interrupt: writing a 1 clears the IRQ  */
		struct RxTimeout
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits RxDone: */
		/* Packet reception complete interrupt: writing a 1 clears the IRQ  */
		struct RxDone
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits PayloadCrcError: */
		/* Payload CRC error interrupt: writing a 1 clears the IRQ  */
		struct PayloadCrcError
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits ValidHeader: */
		/* Valid header received in Rx: writing a 1 clears the IRQ  */
		struct ValidHeader
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits TxDone: */
		/* FIFO Payload transmission complete interrupt: writing a 1 clears the IRQ  */
		struct TxDone
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits CadDone: */
		/* CAD complete: write to clear: writing a 1 clears the IRQ  */
		struct CadDone
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits FhssChangeChannel: */
		/* FHSS change channel interrupt: writing a 1 clears the IRQ  */
		struct FhssChangeChannel
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits CadDetected: */
		/* Valid Lora signal detected during CAD operation: writing a 1 clears the IRQ  */
		struct CadDetected
		{
			/* MODE rc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register IrqFlags */
	void setIrqFlags(uint8_t value)
	{
		write(IrqFlags::__address, value, 8);
	}
	
	/* Get register IrqFlags */
	uint8_t getIrqFlags()
	{
		return read8(IrqFlags::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG RxNbBytes                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RxNbBytes:
	 * Number of payload bytes of latest packet received
	 */
	struct RxNbBytes
	{
		static const uint16_t __address = 19;
		
		/* Bits RxNbBytes: */
		struct RxNbBytes_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RxNbBytes */
	void setRxNbBytes(uint8_t value)
	{
		write(RxNbBytes::__address, value, 8);
	}
	
	/* Get register RxNbBytes */
	uint8_t getRxNbBytes()
	{
		return read8(RxNbBytes::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                       REG RxHeaderCntValue                                        *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxHeaderCntValue:
	 * Number of valid headers received since last transition into Rx mode.
	 * Header and packet counters are reset in Sleep mode.
	 */
	struct RxHeaderCntValue
	{
		static const uint16_t __address = 20;
		
		/* Bits RxHeaderCntValue: */
		struct RxHeaderCntValue_
		{
			/* MODE r */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RxHeaderCntValue */
	void setRxHeaderCntValue(uint16_t value)
	{
		write(RxHeaderCntValue::__address, value, 16);
	}
	
	/* Get register RxHeaderCntValue */
	uint16_t getRxHeaderCntValue()
	{
		return read16(RxHeaderCntValue::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                       REG RxPacketCntValue                                        *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxPacketCntValue:
	 * Number of valid packets received since last transition into Rx mode.
	 * Header and packet counters are reseted in Sleep mode.
	 */
	struct RxPacketCntValue
	{
		static const uint16_t __address = 22;
		
		/* Bits RxPacketCntValue: */
		struct RxPacketCntValue_
		{
			/* MODE rc */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RxPacketCntValue */
	void setRxPacketCntValue(uint16_t value)
	{
		write(RxPacketCntValue::__address, value, 16);
	}
	
	/* Get register RxPacketCntValue */
	uint16_t getRxPacketCntValue()
	{
		return read16(RxPacketCntValue::__address, 16);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG ModemStat                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG ModemStat:
	 */
	struct ModemStat
	{
		static const uint16_t __address = 24;
		
		/* Bits RxCodingRate: */
		/* Coding rate of last header received */
		struct RxCodingRate
		{
			/* MODE r */
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits ModemClear: */
		struct ModemClear
		{
			/* MODE r */
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits HeaderInfoValid: */
		struct HeaderInfoValid
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits RxOngoing: */
		struct RxOngoing
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits SignalSynchronized: */
		struct SignalSynchronized
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits SignalDetected: */
		struct SignalDetected
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register ModemStat */
	void setModemStat(uint8_t value)
	{
		write(ModemStat::__address, value, 8);
	}
	
	/* Get register ModemStat */
	uint8_t getModemStat()
	{
		return read8(ModemStat::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG PktSnrValue                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PktSnrValue:
	 * Estimation of SNR on last packet received.In two’s compliment format mutiplied by 4.SNRdB = PacketSnrtwos complement-----------------------------------------------------------------------------------4
	 */
	struct PktSnrValue
	{
		static const uint16_t __address = 25;
		
		/* Bits PktSnrValue: */
		struct PktSnrValue_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PktSnrValue */
	void setPktSnrValue(uint8_t value)
	{
		write(PktSnrValue::__address, value, 8);
	}
	
	/* Get register PktSnrValue */
	uint8_t getPktSnrValue()
	{
		return read8(PktSnrValue::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG PktRssiValue                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG PktRssiValue:
	 * RSSI of the latest packet received (dBm)RSSI[dBm] = - 139 + PacketRssi (when SNR >= 0) orRSSI[dBm] = - 139 + PacketRssi + PacketSnr *0.25 (when SNR< 0)
	 */
	struct PktRssiValue
	{
		static const uint16_t __address = 26;
		
		/* Bits PktRssiValue: */
		struct PktRssiValue_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PktRssiValue */
	void setPktRssiValue(uint8_t value)
	{
		write(PktRssiValue::__address, value, 8);
	}
	
	/* Get register PktRssiValue */
	uint8_t getPktRssiValue()
	{
		return read8(PktRssiValue::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG RssiValue                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RssiValue:
	 * Current RSSI value (dBm)RSSI[dBm] = - 139 + Rssi
	 */
	struct RssiValue
	{
		static const uint16_t __address = 27;
		
		/* Bits RssiValue: */
		struct RssiValue_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RssiValue */
	void setRssiValue(uint8_t value)
	{
		write(RssiValue::__address, value, 8);
	}
	
	/* Get register RssiValue */
	uint8_t getRssiValue()
	{
		return read8(RssiValue::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG HopChannel                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG HopChannel:
	 */
	struct HopChannel
	{
		static const uint16_t __address = 28;
		
		/* Bits PllTimeout: */
		/*
		 * PLL failed to lock while attempting a TX/RX/CAD operation
		 * 1 -> PLL did not lock
		 * 0 -> PLL did lock
		 */
		struct PllTimeout
		{
			/* MODE r */
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits CrcOnPayload: */
		/*
		 * CRC Information extracted from the received packet header (Explicit header mode only)
		 * 0 -> Header indicates CRC off
		 * 1 -> Header indicates CRC on
		 */
		struct CrcOnPayload
		{
			/* MODE r */
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits FhssPresentChannel: */
		/* Current value of frequency hopping channel in use. */
		struct FhssPresentChannel
		{
			/* MODE r */
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register HopChannel */
	void setHopChannel(uint8_t value)
	{
		write(HopChannel::__address, value, 8);
	}
	
	/* Get register HopChannel */
	uint8_t getHopChannel()
	{
		return read8(HopChannel::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG ModemConfig1                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG ModemConfig1:
	 */
	struct ModemConfig1
	{
		static const uint16_t __address = 29;
		
		/* Bits Bw: */
		/* Signal bandwidth  */
		struct Bw
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b11000000; // [6,7]
			static const uint8_t BW_125 = 0b00; // 125 kHz
			static const uint8_t BW_250 = 0b01; // 250 kHz
			static const uint8_t BW_500 = 0b10; // 500 kHz
			static const uint8_t reserved_0 = 0b11; // 
		};
		/* Bits CodingRate: */
		/*
		 * Error coding rate
		 * All other values -> reserved
		 * In implicit header mode should be set on receiver to determine expected coding rate. See Section 4.1.1.3.
		 */
		struct CodingRate
		{
			/* MODE rw */
			static const uint8_t dflt = 0b001; // 3'b1
			static const uint8_t mask = 0b00111000; // [3,4,5]
			static const uint8_t CR_4_5 = 0b01; // 4/5
			static const uint8_t CR_4_6 = 0b10; // 4/6
			static const uint8_t CR_4_7 = 0b11; // 4/7
			static const uint8_t CR_4_8 = 0b100; // 4/8
		};
		/* Bits ImplicitHeaderModeOn: */
		/*
		 * 0 -> Explicit Header mode
		 * 1 -> Implicit Header mode
		 */
		struct ImplicitHeaderModeOn
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits RxPayloadCrcOn: */
		/*
		 * Enable CRC generation and check on payload:
		 * 0 ->CRC disable
		 * 1 ->CRC enable
		 * If CRC is needed, RxPayloadCrcOn should be set:
		 * in Implicit header mode: on Tx and Rx side
		 * in Explicit header mode: on the Tx side alone (recovered from the header in Rx side)
		 */
		struct RxPayloadCrcOn
		{
			/* MODE rw */
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits LowDataRateOptimize: */
		/*
		 * 0 -> Disabled
		 * 1 -> Enabled
		 * mandated for SF11 and SF12 with BW = 125 kHz
		 */
		struct LowDataRateOptimize
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register ModemConfig1 */
	void setModemConfig1(uint8_t value)
	{
		write(ModemConfig1::__address, value, 8);
	}
	
	/* Get register ModemConfig1 */
	uint8_t getModemConfig1()
	{
		return read8(ModemConfig1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG ModemConfig2                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG ModemConfig2:
	 */
	struct ModemConfig2
	{
		static const uint16_t __address = 30;
		
		/* Bits SpreadingFactor: */
		/* SF rate (expressed as a base-2 logarithm) 6, ..., 12. Other values reserved.  */
		struct SpreadingFactor
		{
			/* MODE rw */
			static const uint16_t dflt = 0b0111; // 4'h7
			static const uint16_t mask = 0b1111000000000000; // [12,13,14,15]
			static const uint16_t SF_64 = 6; // 64 chips / symbol
			static const uint16_t SF_128 = 7; // 128 chips / symbol
			static const uint16_t SF_256 = 8; // 256 chips / symbol
			static const uint16_t SF_512 = 9; // 512 chips / symbol
			static const uint16_t SF_1024 = 10; // 1024 chips / symbol
			static const uint16_t SF_2048 = 11; // 2048 chips / symbol
			static const uint16_t SF_4096 = 12; // 4096 chips / symbol
		};
		/* Bits TxContinuousMode: */
		/*
		 * 0 -> normal mode, a single packet is sent
		 * 1 -> continuous mode, send multiple packets across the FIFO (used for spectral analysis)
		 */
		struct TxContinuousMode
		{
			/* MODE rw */
			static const uint16_t dflt = 0b0; // 1'b0
			static const uint16_t mask = 0b0000100000000000; // [11]
		};
		/* Bits AgcAutoOn: */
		/* 0 -> LNA gain set by register LnaGain1 -> LNA gain set by the internal AGC loop  */
		struct AgcAutoOn
		{
			/* MODE rw */
			static const uint16_t dflt = 0b1; // 1'b1
			static const uint16_t mask = 0b0000010000000000; // [10]
		};
		/* Bits SymbTimeout: */
		/*
		 * RX Time-Out LSBRX operation time-out value expressed as number of symbols:
		 * TimeOut = SymbTimeout * Ts
		 */
		struct SymbTimeout
		{
			/* MODE rw */
			static const uint16_t dflt = 0b0001100100; // 10'h64
			static const uint16_t mask = 0b0000001111111111; // [0,1,2,3,4,5,6,7,8,9]
		};
	};
	
	/* Set register ModemConfig2 */
	void setModemConfig2(uint16_t value)
	{
		write(ModemConfig2::__address, value, 16);
	}
	
	/* Get register ModemConfig2 */
	uint16_t getModemConfig2()
	{
		return read16(ModemConfig2::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG Preamble                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Preamble:
	 * Preamble length = PreambleLength + 4.25 Symbols
	 * See Section 4.1.1.6 for more details.
	 */
	struct Preamble
	{
		static const uint16_t __address = 32;
		
		/* Bits Preamble: */
		struct Preamble_
		{
			/* MODE rw */
			static const uint16_t dflt = 0b0000000000001000; // 16'h8
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register Preamble */
	void setPreamble(uint16_t value)
	{
		write(Preamble::__address, value, 16);
	}
	
	/* Get register Preamble */
	uint16_t getPreamble()
	{
		return read16(Preamble::__address, 16);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG PayloadLength                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PayloadLength:
	 * Payload length in bytes. The register needs to be set in implicit header mode for the expected packet length.
	 * A 0 value is not permitted.
	 */
	struct PayloadLength
	{
		static const uint16_t __address = 34;
		
		/* Bits PayloadLength: */
		struct PayloadLength_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000001; // 8'h1
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PayloadLength */
	void setPayloadLength(uint8_t value)
	{
		write(PayloadLength::__address, value, 8);
	}
	
	/* Get register PayloadLength */
	uint8_t getPayloadLength()
	{
		return read8(PayloadLength::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                       REG MaxPayloadLength                                        *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG MaxPayloadLength:
	 * Maximum payload length; if header payload length exceeds value a header CRC error is generated.
	 * Allows filtering of packet with a bad size.
	 */
	struct MaxPayloadLength
	{
		static const uint16_t __address = 35;
		
		/* Bits MaxPayloadLength: */
		struct MaxPayloadLength_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b11111111; // 8'hff
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register MaxPayloadLength */
	void setMaxPayloadLength(uint8_t value)
	{
		write(MaxPayloadLength::__address, value, 8);
	}
	
	/* Get register MaxPayloadLength */
	uint8_t getMaxPayloadLength()
	{
		return read8(MaxPayloadLength::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG HopPeriod                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG HopPeriod:
	 * Symbol periods between frequency hops. (0 = disabled). 1st hop always happen after the 1st header symbol.
	 */
	struct HopPeriod
	{
		static const uint16_t __address = 36;
		
		/* Bits HopPeriod: */
		struct HopPeriod_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register HopPeriod */
	void setHopPeriod(uint8_t value)
	{
		write(HopPeriod::__address, value, 8);
	}
	
	/* Get register HopPeriod */
	uint8_t getHopPeriod()
	{
		return read8(HopPeriod::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG FifoRxByteAddr                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FifoRxByteAddr:
	 * Current value of RX databuffer pointer (address of last byte written by Lora receiver)
	 */
	struct FifoRxByteAddr
	{
		static const uint16_t __address = 37;
		
		/* Bits FifoRxByteAddr: */
		struct FifoRxByteAddr_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoRxByteAddr */
	void setFifoRxByteAddr(uint8_t value)
	{
		write(FifoRxByteAddr::__address, value, 8);
	}
	
	/* Get register FifoRxByteAddr */
	uint8_t getFifoRxByteAddr()
	{
		return read8(FifoRxByteAddr::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Fei                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Fei:
	 */
	struct Fei
	{
		static const uint16_t __address = 40;
		
		/* Bits Reserved_0: */
		struct Reserved_0
		{
			static const uint32_t mask = 0b111100000000000000000000; // [20,21,22,23]
		};
		/* Bits FreqError: */
		/*
		 * Estimated frequency error from modem in 2’s complement format.
		 * F_error = FreqError * 2^24 / F_xtal
		 */
		struct FreqError
		{
			/* MODE r */
			static const uint32_t dflt = 0b00000000000000000000; // 20'h0
			static const uint32_t mask = 0b000011111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
		};
	};
	
	/* Set register Fei */
	void setFei(uint32_t value)
	{
		write(Fei::__address, value, 24);
	}
	
	/* Get register Fei */
	uint32_t getFei()
	{
		return read32(Fei::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG RssiWideband                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RssiWideband:
	 * Wideband RSSI measurement used to locally generate a random number
	 */
	struct RssiWideband
	{
		static const uint16_t __address = 44;
		
		/* Bits RssiWideband: */
		struct RssiWideband_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RssiWideband */
	void setRssiWideband(uint8_t value)
	{
		write(RssiWideband::__address, value, 8);
	}
	
	/* Get register RssiWideband */
	uint8_t getRssiWideband()
	{
		return read8(RssiWideband::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG DetectOptimize                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG DetectOptimize:
	 */
	struct DetectOptimize
	{
		static const uint16_t __address = 49;
		
		/* Bits Reserved_0: */
		struct Reserved_0
		{
			static const uint8_t dflt = 0b11000; // 5'b11000
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits DetectionOptimize: */
		/* LoRa detection Optimize  */
		struct DetectionOptimize
		{
			/* MODE rw */
			static const uint8_t dflt = 0b011; // 3'b11
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t SF7_12 = 0b11; // SF7 to SF12
			static const uint8_t SF6 = 0b101; // 
		};
	};
	
	/* Set register DetectOptimize */
	void setDetectOptimize(uint8_t value)
	{
		write(DetectOptimize::__address, value, 8);
	}
	
	/* Get register DetectOptimize */
	uint8_t getDetectOptimize()
	{
		return read8(DetectOptimize::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG InvertIQ                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG InvertIQ:
	 */
	struct InvertIQ
	{
		static const uint16_t __address = 51;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits InvertIQ: */
		/*
		 * Invert the LoRa I and Q signals:
		 * 0 -> normal mode
		 * 1 -> I and Q signals are inverted
		 */
		struct InvertIQ_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits Reserved_1: */
		struct Reserved_1
		{
			static const uint8_t dflt = 0b100111; // 6'b100111
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register InvertIQ */
	void setInvertIQ(uint8_t value)
	{
		write(InvertIQ::__address, value, 8);
	}
	
	/* Get register InvertIQ */
	uint8_t getInvertIQ()
	{
		return read8(InvertIQ::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                      REG DetectionThreshold                                       *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG DetectionThreshold:
	 * LoRa detection threshold
	 */
	struct DetectionThreshold
	{
		static const uint16_t __address = 55;
		
		/* Bits Threshold: */
		struct Threshold
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00001010; // 8'b1010
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
			static const uint8_t SF7_12 = 0xa; // SF7 to SF12
			static const uint8_t SF6 = 0xc; // SF6
		};
	};
	
	/* Set register DetectionThreshold */
	void setDetectionThreshold(uint8_t value)
	{
		write(DetectionThreshold::__address, value, 8);
	}
	
	/* Get register DetectionThreshold */
	uint8_t getDetectionThreshold()
	{
		return read8(DetectionThreshold::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG SyncWord                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncWord:
	 * LoRa Sync WordValue 0x34 is reserved for LoRaWAN networks
	 */
	struct SyncWord
	{
		static const uint16_t __address = 57;
		
		/* Bits SyncWord: */
		struct SyncWord_
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00010010; // 8'h12
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncWord */
	void setSyncWord(uint8_t value)
	{
		write(SyncWord::__address, value, 8);
	}
	
	/* Get register SyncWord */
	uint8_t getSyncWord()
	{
		return read8(SyncWord::__address, 8);
	}
	
};
