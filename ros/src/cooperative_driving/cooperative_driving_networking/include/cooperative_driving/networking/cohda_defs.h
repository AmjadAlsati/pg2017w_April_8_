//////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, CCS Labs
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////
/*
 * Includes definition of cohdas structures and datatypes so that they can also be used
 * with UNEX devices
 *
 */

#ifndef COHDA_DEFS_H_
#define COHDA_DEFS_H_

/// The 802.11 channel number, per 802.11-2007
typedef uint8_t tMK2ChannelNumber;

/**
 * Channel type
 * Indicates CCH or SCH
 */
typedef enum Mk2Channel
{
  /// CCH
  MK2CHAN_CCH = 0x01,
  /// SCH
  MK2CHAN_SCH = 0x80,

  /// CCH on radio A
  MK2CHAN_CCH_A = 0x01,
  /// SCH on radio A
  MK2CHAN_SCH_A = 0x08,

  /// CCH on radio B
  MK2CHAN_CCH_B = 0x10,
  /// SCH on radio B
  MK2CHAN_SCH_B = 0x80,

  /// CCH (interval 1) on radio A
  MK2CHAN_CCH_A1 = 0x01,
  /// CCH (interval 2) on radio A
  MK2CHAN_CCH_A2 = 0x02,
  /// SCH (interval 1) on radio A
  MK2CHAN_SCH_A1 = 0x08,
  /// SCH (interval 2) on radio A
  MK2CHAN_SCH_A2 = 0x04,

} eMK2Channel;
/// @copydoc eMK2Channel
typedef uint8_t tMK2Channel;

/**
 * MK2 Bandwidth
 * Indicates 10 MHz or 20 MHz
 */
typedef enum MK2Bandwidth
{
  /// Indicates 10 MHz
  MK2BW_10MHz,
  /// Indicates 20 MHz
  MK2BW_20MHz
} eMK2Bandwidth;
/// @copydoc eMK2Bandwidth
typedef uint8_t tMK2Bandwidth;

/**
 * MK2 dual radio transmit control
 * Controls transmit behaviour according to activity on the
 * other radio (inactive in single radio configurations)
 */
typedef enum MK2DualTxControl
{
  /// Do not constrain transmissions
  MK2TXC_NONE,
  /// Prevent transmissions when other radio is transmitting
  MK2TXC_TX,
  /// Prevent transmissions when other radio is receiving
  MK2TXC_RX,
  /// Prevent transmissions when other radio is transmitting or receiving
  MK2TXC_TXRX,
  /// Default behaviour
  MK2TXC_DEFAULT = MK2TXC_TX,

} eMK2DualTxControl;
/// @copydoc eMK2DualTxControl
typedef uint8_t tMK2DualTxControl;

/**
 * MK2 Modulation and Coding scheme
 */
typedef enum MK2MCS
{
  /// Rate 1/2 BPSK
  MK2MCS_R12BPSK = 0xB,
  /// Rate 3/4 BPSK
  MK2MCS_R34BPSK = 0xF,
  /// Rate 1/2 QPSK
  MK2MCS_R12QPSK = 0xA,
  /// Rate 3/4 QPSK
  MK2MCS_R34QPSK = 0xE,
  /// Rate 1/2 16QAM
  MK2MCS_R12QAM16 = 0x9,
  /// Rate 3/4 16QAM
  MK2MCS_R34QAM16 = 0xD,
  /// Rate 2/3 64QAM
  MK2MCS_R23QAM64 = 0x8,
  /// Rate 3/4 64QAM
  MK2MCS_R34QAM64 = 0xC,
  /// Use default data rate
  MK2MCS_DEFAULT = 0x0,
  /// Use transmit rate control
  MK2MCS_TRC = 0x1,
} eMK2MCS;
/// @copydoc eMK2MCS
typedef uint8_t tMK2MCS;

/**
 * MK2 Power
 * Indicates the power in 0.5 dBm steps (signed to indicate Rx power too)
 */
typedef int16_t tMK2Power;

/// Indicate manual, default or TPC
typedef enum MK2TxPwrCtl
{
  /// Manually specified
  MK2TPC_MANUAL,
  /// Default values
  MK2TPC_DEFAULT,
  /// Utilize dynamic TPC
  MK2TPC_TPC
} eMK2TxPwrCtl;
/// @copydoc eMK2TxPwrCtl
typedef uint8_t tMK2TxPwrCtl;

/**
 * MK2 Transmit Power
 * Indicates if manual, default or TPC is to be used, and manual power
 * setting if needed. If the power specified is lower than the minimum power supported by the
 * hardware, the minimum will be used. If the power specified is higher than the maximum power
 * supported by the hardware, the maximum will be used.
 */
typedef struct MK2TxPower
{
  /// Indicate manual, default or TPC
  tMK2TxPwrCtl PowerSetting;
  /// The manual power setting (if used)
  tMK2Power ManualPower;
}__attribute__ ((packed)) tMK2TxPower;

/**
 * MK2 Transmit Antenna
 * Indicates if manual, or automatic antenna control is to
 * be used, and a manual antenna setting if needed.
 */
typedef enum MK2TxAntenna
{
  /// Transmit packet using automatic/default transmit antenna selection.
  MK2_TXANT_DEFAULT = 0,
  /// Transmit packet on antenna 1
  MK2_TXANT_ANTENNA1 = 1,
  /// Transmit packet on antenna 2 (when available).
  MK2_TXANT_ANTENNA2 = 2,
  /// Transmit packet on both antenna
  MK2_TXANT_ANTENNA1AND2 = 3
} eMK2TxAntenna;
/// @copydoc eMK2TxAntenna
typedef uint8_t tMK2TxAntenna;

/**
 * MK2 Receive Antenna
 * Indicates if manual, or automatic antenna control is to
 * be used, and manual antenna setting if needed.
 */
typedef enum MK2RxAntenna
{
  /// Receive using default antenna selection (default is MRC)
  MK2_RXANT_DEFAULT = 0,
  /// Receive only on antenna 1
  MK2_RXANT_ANTENNA1 = 1,
  /// Receive only on antenna 2 (when available).
  MK2_RXANT_ANTENNA2 = 2,
  /// Receive on both antennas (MRC)
  MK2_RXANT_ANTENNA1AND2 = 3
} eMK2RxAntenna;
/// @copydoc eMK2RxAntenna
typedef uint8_t tMK2RxAntenna;

/**
 * MK2 TSF
 * Indicates absolute 802.11 MAC time in micro seconds
 */
typedef uint64_t tMK2TSF;

/**
 * MK2 Expiry time
 * Indicates absolute 802.11 MAC time in micro seconds
 */
typedef uint64_t tMK2Time;

/**
 * MK2 MAC Address
 */
typedef uint8_t tMK2MACAddr[6];

/**
 * MK2 Rate sets
 * Each bit indicates if corresponding MCS rate is supported
 */
typedef enum MK2Rate
{
  /// Rate 1/2 BPSK rate mask
  MK2_RATE12BPSK_MASK = 0x01,
  /// Rate 3/4 BPSK rate mask
  MK2_RATE34BPSK_MASK = 0x02,
  /// Rate 1/2 QPSK rate mask
  MK2_RATE12QPSK_MASK = 0x04,
  /// Rate 3/4 QPSK rate mask
  MK2_RATE34QPSK_MASK = 0x08,
  /// Rate 1/2 16QAM rate mask
  MK2_RATE12QAM16_MASK = 0x10,
  /// Rate 2/3 64QAM rate mask
  MK2_RATE23QAM64_MASK = 0x20,
  /// Rate 3/4 16QAM rate mask
  MK2_RATE34QAM16_MASK = 0x40,
} eMK2Rate;
/// @copydoc eMK2Rate
typedef uint8_t tMK2Rate;

/**
 * MK2 Priority
 */
typedef enum MK2Priority
{
  /// Priority level 0
  MK2_PRIO_0 = 0,
  /// Priority level 1
  MK2_PRIO_1 = 1,
  /// Priority level 2
  MK2_PRIO_2 = 2,
  /// Priority level 3
  MK2_PRIO_3 = 3,
  /// Priority level 4
  MK2_PRIO_4 = 4,
  /// Priority level 5
  MK2_PRIO_5 = 5,
  /// Priority level 6
  MK2_PRIO_6 = 6,
  /// Priority level 7
  MK2_PRIO_7 = 7,
} eMK2Priority;
/// @copydoc eMK2Priority
typedef uint8_t tMK2Priority;

/**
 * MK2 802.11 service class specification.
 */
typedef enum MK2Service
{
  /// Packet should be (was) transmitted using normal ACK policy
  MK2_QOS_ACK = 0x00,
  /// Packet should be (was) transmitted without Acknowledgement.
  MK2_QOS_NOACK = 0x01
} eMK2Service;
/// @copydoc eMK2Service
typedef uint8_t tMK2Service;



#endif /* COHDA_DEFS_H_ */
