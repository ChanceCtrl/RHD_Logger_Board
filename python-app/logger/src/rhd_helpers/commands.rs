#[derive(Debug, Clone, Copy)]
pub enum Commands {
    Convert,
    Calibrate,
    ClearCalibration,
    WriteRegister,
    ReadRegister,
}

pub trait ReadableRegister {
    fn address(&self) -> u8;
}

#[derive(Debug, Clone, Copy)]
pub enum ConfigRegisters {
    // Register 0: ADC Configuration and Amplifier Fast Settle
    //
    // ADC reference BW [1:0]: This variable configures the bandwidth of an internal ADC
    // reference generator feedback circuit. This variable should always be set to 3.
    //
    // amp fast settle: Setting this bit to one closes a switch in each amplifier that
    // drives its analog output to the baseline “zero” level. This can be used to quickly
    // recover from large transient events that may drive the amplifiers to their rails. The
    // switch should be closed for a certain amount of time to settle the amplifiers
    // (see “Fast Settle Function” section for details) and then this register should be reset
    // to zero to resume normal amplifier operation.
    //
    // amp Vref enable: In normal operation, this bit should be set to one to power up voltage
    // references used by the biopotential amplifiers. This bit can be set to zero to reduce power
    // supply current consumption by 180 µA when the amplifiers will not be used for an extended period
    // of time. After setting this bit to one, at least 100 µs must elapse before ADC samples are valid,
    // or before ADC calibration is executed.
    //
    // ADC comparator bias [1:0]: This variable configures the bias current of the ADC comparator.
    // This variable should always be set to 3 for normal operation and ADC calibration. This variable can
    // be set to zero to reduce power supply current consumption by 80 µA when the ADC will not be used
    // for an extended period of time.
    //
    // ADC comparator select [1:0]: This variable selects between four different comparators that
    // can be used by the ADC. This variable should always be set to 2.
    //
    // | D[7:6]             | D[5]        | D[4]        | D[3:2]                | D[1:0]                  |
    // | reference BW [1:0] | fast settle | Vref enable | comparator bias [1:0] | comparator select [1:0] |
    ADCConfiguration = 0,

    // Register 1: Supply Sensor and ADC Buffer Bias Current
    //
    // VDD sense enable: Setting this bit to one enables the on-chip supply voltage sensor, whose output
    // may be sampled by the ADC on channel 48 (see “Supply Voltage Sensor” section for details). If the
    // supply voltage is not sampled, this bit can be set to zero to reduce current consumption by 10 µA.
    //
    // ADC buffer bias [5:0]: This variable configures the bias current of an internal reference buffer
    // in the ADC. The optimum value for this variable is a function of ADC sampling rate and is listed
    // in a table in the “Analog-to-Digital Converter” section later in the datasheet.
    //
    // | D[7] | D[6]             | D[5:0]                |
    // | X    | VDD sense enable | ADC buffer bias [5:0] |
    SupplySensor_ADCBufferBiasCurrent = 1,

    // Register 2: MUX Bias Current
    //
    // MUX bias [5:0]: This variable configures the bias current of the MUX that routes the selected
    // analog signal to the ADC input. The optimum value for this variable is a function of ADC sampling
    // rate and is listed in a table in the “Analog-to-Digital Converter” section later in the datasheet.
    //
    // | D[7] | D[6] | D[5:0]         |
    // | X    | X    | MUX bias [5:0] |
    MUXBiasCurrent = 2,

    // Register 3: MUX Load, Temperature Sensor, and Auxiliary Digital Output
    //
    // MUX load [2:0]: This variable configures the total capacitance at the input of the ADC.
    // This variable should always be set to 0.
    //
    // tempS1 and tempS2: These bits control switches in the on-chip temperature sensor, whose output may
    // be sampled by the ADC on channel 49. The detailed operation of the temperature sensor is described
    // in the “Temperature Sensor” section later in the datasheet. When the temperature sensor is not in
    // use, these bits should each be set to zero to save power.
    //
    // tempen: Setting this bit to one enables the on-chip temperature sensor. Current consumption may be
    // reduced by approximately 70 µA by setting this bit to zero to disable the sensor.
    //
    // digout HiZ: The RHD2000 chips have an auxiliary digital output pin auxout that may be used to
    // activate off-chip circuitry (e.g., MOSFET switches, LEDs, stimulation circuits). Setting this bit
    // to one puts the digital output into high impedance (HiZ) mode.
    //
    // digout: This bit is driven out of the auxiliary CMOS digital output pin auxout, provided that the
    // digout HiZ bit is set to zero. See the “Auxiliary Digital Output” section for details.
    //
    // | D[7:5]         | D[4]   | D[3]   | D[2]   | D[1]       | D[0]   |
    // | MUX load [2:0] | tempS2 | tempS1 | tempen | digout HiZ | digout |
    MUXLoad_TemperatureSensor_AuxiliaryDigitalOutput = 3,

    // Register 4: ADC Output Format and DSP Offset Removal
    //
    // weak MISO: If this bit is set to zero, the MISO line goes to high impedance mode (HiZ) when CS is
    // pulled high, allowing multiple chips to share the same MISO line so long as only one of their chip
    // select lines is activated at any time. If only one RHD2000 chip will be using a MISO line, this bit
    // may be set to one, and when CS is pulled high the MISO line will be driven weakly by the chip.
    // This can prevent the line from drifting to indeterminate values between logic high and logic low.
    //
    // twoscomp: If this bit is set to one, amplifier conversions from the ADC are reported using a
    // “signed” two’s complement representation where the amplifier baseline is reported as zero and values
    // below baseline are reported as negative numbers. If this bit is set to zero, amplifier conversions
    // from the ADC are reported using “unsigned” offset binary notation where the baseline level is
    // represented as 1000000000000000. ADC conversions from non-amplifier channels (i.e., C > 31) are
    // always reported as unsigned binary numbers.
    //
    // absmode: Setting this bit to one passes all amplifier ADC conversions through an absolute value
    // function. This is equivalent to performing full-wave rectification on the signals, and may be
    // useful for implementing symmetric positive/negative thresholds or envelope estimation algorithms.
    // This bit has no effect on ADC conversions from non-amplifier channels (i.e., C > 31). See the
    // “Absolute Value Mode” section for more information.
    //
    // DSPen: When this bit is set to one, the RHD2000 performs digital signal processing (DSP) offset
    // removal from all 32 amplifier channels using a first-order high-pass IIR filter. See the
    // “DSP High-Pass Filter for Offset Removal” section for details.
    //
    // DSP cutoff freq [3:0]: This variable sets the cutoff frequency of the DSP filter used to for offset
    // removal. See the “DSP HighPass Filter for Offset Removal” section for details.
    //
    // | D[7]      | D[6]     | D[5]    | D[4]  | D[3:0]                |
    // | weak MISO | twoscomp | absmode | DSPen | DSP cutoff freq [3:0] |
    ADCOutputFormat_DSPOffsetRemoval = 4,

    // Register 5: Impedance Check Control
    //
    // Zcheck DAC power: Setting this bit to one activates the on-chip digital-to-analog converter (DAC)
    // used to generate waveforms for electrode impedance measurement. If impedance testing is not being
    // performed, this bit can be set to zero to reduce current consumption by 120 µA. See the “On-Chip
    // AC Current Waveform Generator” section for more information.
    //
    // Zcheck load: Setting this bit to one adds a capacitor load to the impedance checking network.
    // This mode is only used for chip testing at Intan Technologies. This bit should always be set to
    // zero for normal operation.
    //
    // Zcheck scale [1:0]: This variable selects the series capacitor used to convert the voltage waveform
    // generated by the on-chip DAC into an AC current waveform that stimulates a selected electrode for
    // impedance testing: 00 = 0.1 pF; 01 = 1.0 pF; 11 = 10 pF. See the “On-Chip AC Current Waveform
    // Generator” section for more information.
    //
    // Zcheck conn all: Setting this bit to one connects all electrodes together to the elec_test input
    // pin. This is only used for applying DC voltages to electroplate electrodes. In normal operation
    // this bit should be set to zero. See the “Electrode Activation” section for details.
    //
    // Zcheck sel pol: This bit is only used on the RHD2216 where the biopotential amplifiers have separate
    // positive and negative inputs (instead of a reference input common to all amplifiers). Setting this
    // bit to zero selects impedance testing of the positive input of the selected amplifier. Setting the
    // bit to one tests the negative input. See the “Electrode Impedance Test” section for details.
    //
    // Zcheck en: Setting this bit to one activates impedance testing mode, and connects the on-chip waveform
    // generator (and pin elec_test) to the amplifier selected by the Zcheck select variable in Register 7.
    // See the “Electrode Impedance Test” section for details.
    //
    // | D[7] | D[6]             | D[5]        | D[4:3]      | D[2]     | D[1]    | D[0]      |
    // | X    | Zcheck DAC power | Zcheck load | scale [1:0] | conn all | sel pol | Zcheck en |
    ImpedanceCheckControl = 5,

    // Register 6: Impedance Check DAC
    //
    // Zcheck DAC [7:0]: This variable sets the output voltage of an 8-bit DAC used to generate waveforms
    // for impedance checking. This variable must be updated at regular intervals to create the desired
    // waveform. Note that this DAC must be enabled by setting Zcheck DAC power in Register 5. If
    // impedance testing is not in progress, the value of this register should remain unchanged to
    // minimize noise (although writing the same value to the register is acceptable). See the “On-Chip
    // AC Current Waveform Generator” section for more information.
    //
    // | D[7:0]           |
    // | Zcheck DAC [7:0] |
    ImpedanceCheckDAC = 6,

    // Zcheck select [5:0]: This variable selects the amplifier whose electrode will be connected to the
    // on-chip impedance testing circuitry if Zcheck en is set to one. In the RHD2164, all six bits of this
    // register are used. Although the 64 amplifiers are divided between two analog multiplexers and are
    // sampled by the two on-chip ADCs in pairs (e.g., channel 0 and channel 32 are sampled simultaneously
    // when a CONVERT(0) command is sent), impedance checking is performed only on one channel at a time.
    //
    // | D[7] | D[6] | D[5:0]              |
    // | X    | X    | Zcheck select [5:0] |
    ImpedanceCheckAmplifierSelect = 7,

    // apwr [31:0]: Setting these bits to zero powers down individual biopotential amplifiers, saving power
    // if there are channels that don’t need to be observed. Each amplifier consumes power in proportion to
    // its upper cutoff frequency. Current consumption is approximately 7.6 µA/kHz per amplifier. Under
    // normal operation, these bits should be set to one.
    //
    // | D[7]    | D[6]    | D[5]    | D[4]    | D[3]    | D[2]    | D[1]    | D[0]    |
    // | apwr[7] | apwr[6] | apwr[5] | apwr[4] | apwr[3] | apwr[2] | apwr[1] | apwr[0] |
    IndividualAmplifierPower7_0 = 14,
    IndividualAmplifierPower15_8 = 15,
    IndividualAmplifierPower23_16 = 16,
    IndividualAmplifierPower31_24 = 17,

    // apwr [32:63]: Registers 18-21 do not exist on the RHD2216 or RHD2132 chips. The contents of these
    // RHD2164-specific registers may be set using the traditional WRITE command. However, if a READ command
    // is sent to read back the contents of Registers 18-21, the results will only appear correctly on the
    // MISO B data stream. The results of all other registers should be read from the MISO A data stream. In
    // most applications it is probably not necessary to read the contents of these registers; when they are
    // modified, copies of their most recent values may be stored on the controller.
    //
    // | D[7]     | D[6]     | D[5]     | D[4]     | D[3]     | D[2]     | D[1]     | D[0]     |
    // | apwr[32] | apwr[33] | apwr[34] | apwr[35] | apwr[36] | apwr[37] | apwr[38] | apwr[39] |
    IndividualAmplifierPower32_39 = 18,
    IndividualAmplifierPower40_47 = 19,
    IndividualAmplifierPower48_55 = 20,
    IndividualAmplifierPower56_63 = 21,
    // NOTE: I have no fucking clue why, but for the upper range of registers it flips the order of
    // bit to apwr value... This fucking company hates people I swear.
}

impl ReadableRegister for ConfigRegisters {
    fn address(&self) -> u8 {
        return *self as u8;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ReadOnlyRegisters {
    // The read-only registers 40-44 contain the characters INTAN in ASCII.
    // The contents of these registers can be read to verify the fidelity of the SPI interface.
    CompanyDesignatorI = 40,
    CompanyDesignatorN = 41,
    CompanyDesignatorT = 42,
    CompanyDesignatorA = 43,

    // This read-only variable returns 00110101 (decimal 53) on MISO A and 00111010 (decimal 58) on MISO B.
    // These distinct bytes can be checked by the SPI master device to confirm signal integrity on the SPI
    // bus (e.g., to adjust internal sampling times to compensate for cable propagation delay).
    MISOABMarker = 59,

    // This read-only variable encodes a die revision number which is set
    // by Intan Technologies to encode various versions of a chip
    DieRevision = 60,

    // This read-only variable is set to zero if the on-chip biopotential amplifiers have
    // independent differential (bipolar) inputs like the RHD2216 chip. It is set to one
    // if the amplifiers have unipolar inputs and a common reference, like the RHD2132 chip.
    AmplifierType = 61,

    // This read-only variable encodes the total number of biopotential amplifiers on the chip (e.g., 16, 32).
    AmplifierCount = 62,

    // This read-only variable encodes a unique Intan Technologies ID number indicating the
    // type of chip. The chip ID for the RHD2132 is 1. The chip ID for the RHD2216 is 2.
    ChipID = 63,
}

impl ReadableRegister for ReadOnlyRegisters {
    fn address(&self) -> u8 {
        return *self as u8;
    }
}
