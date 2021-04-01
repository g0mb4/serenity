/*
 * Copyright (c) 2018-2020, Andreas Kling <kling@serenityos.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//
// Parallel ATA (PATA) controller driver
//
// This driver describes a logical PATA Channel. Each channel can connect up to 2
// IDE Hard Disk Drives. The drives themselves can be either the master drive (hd0)
// or the slave drive (hd1).
//
// More information about the ATA spec for PATA can be found here:
//      ftp://ftp.seagate.com/acrobat/reference/111-1c.pdf
//
#pragma once

// 82077AA CHMOS single-chip floppy disk controller

#include <AK/OwnPtr.h>
#include <AK/RefPtr.h>
#include <Kernel/Devices/Device.h>
#include <Kernel/IO.h>
#include <Kernel/Interrupts/IRQHandler.h>
#include <Kernel/Lock.h>
#include <Kernel/PhysicalAddress.h>
#include <Kernel/Random.h>
#include <Kernel/Storage/StorageDevice.h>
#include <Kernel/VM/PhysicalPage.h>
#include <Kernel/WaitQueue.h>

#define FLOPPY_DRIVE_TYPE_NO_DRIVE      0
#define FLOPPY_DRIVE_TYPE_360KB_525IN   1
#define FLOPPY_DRIVE_TYPE_1200KB_525IN  2
#define FLOPPY_DRIVE_TYPE_720KB_35IN    3
#define FLOPPY_DRIVE_TYPE_1440KB_35IN   4
#define FLOPPY_DRIVE_TYPE_2880KB_35IN   5

#define FDC_DOR_MASK_DRIVE0			0x00
#define FDC_DOR_MASK_DRIVE1			0x01
#define FDC_DOR_MASK_DRIVE2			0x02
#define FDC_DOR_MASK_DRIVE3			0x03
#define FDC_DOR_MASK_RESET			0x04
#define FDC_DOR_MASK_DMA			0x08
#define FDC_DOR_MASK_DRIVE0_MOTOR	0x10
#define FDC_DOR_MASK_DRIVE1_MOTOR	0x20
#define FDC_DOR_MASK_DRIVE2_MOTOR	0x40
#define FDC_DOR_MASK_DRIVE3_MOTOR	0x80

#define FDC_MSR_MASK_DRIVE1_POS_MODE	0x01
#define FDC_MSR_MASK_DRIVE2_POS_MODE	0x02
#define FDC_MSR_MASK_DRIVE3_POS_MODE	0x04
#define FDC_MSR_MASK_DRIVE4_POS_MODE	0x08
#define FDC_MSR_MASK_BUSY				0x10
#define FDC_MSR_MASK_DMA				0x20
#define FDC_MSR_MASK_DATAIO				0x40
#define FDC_MSR_MASK_DATAREG			0x80

#define FDC_ST0_MASK_DRIVE0			0x00
#define FDC_ST0_MASK_DRIVE1			0x01
#define FDC_ST0_MASK_DRIVE2			0x02
#define FDC_ST0_MASK_DRIVE3			0x03
#define FDC_ST0_MASK_HEADACTIVE		0x04
#define FDC_ST0_MASK_NOTREADY		0x08
#define FDC_ST0_MASK_UNITCHECK		0x10
#define FDC_ST0_MASK_SEEKEND		0x20
#define FDC_ST0_MASK_INTCODE		0x40

#define FDC_ST0_TYP_NORMAL			0x00
#define FDC_ST0_TYP_ABNORMAL_ERR	0x01
#define FDC_ST0_TYP_INVALID_ERR		0x02
#define FDC_ST0_TYP_NOTREADY		0x03

#define FDC_GAP3_LENGTH_STD 	42
#define FDC_GAP3_LENGTH_5_14	32
#define FDC_GAP3_LENGTH_3_5		27

#define FDC_SECTOR_DTL_128		0x00
#define FDC_SECTOR_DTL_256		0x01
#define FDC_SECTOR_DTL_512		0x02
#define FDC_SECTOR_DTL_1024		0x04

#define	FDC_CMD_READ_TRACK		0x02
#define	FDC_CMD_SPECIFY			0x03
#define	FDC_CMD_CHECK_STAT		0x04
#define	FDC_CMD_WRITE_SECT		0x05
#define	FDC_CMD_READ_SECT		0x06
#define	FDC_CMD_CALIBRATE		0x07
#define	FDC_CMD_check_interrupt		0x08
#define	FDC_CMD_FORMAT_TRACK	0x0D
#define	FDC_CMD_SEEK			0x0F
#define FDC_CMD_VERSION         0x10
#define FDC_CMD_PERPENDICULAR   0x12
#define FDC_CMD_CONFIGURE       0x13
#define FDC_CMD_LOCK            0x94
#define FDC_CMD_UNLOCK          0x14

#define FDC_CMD_EXT_SKIP		0x20
#define FDC_CMD_EXT_DENSITY		0x40
#define FDC_CMD_EXT_MULTITRACK	0x80

#define FDC_SECTORS_PER_TRACK	18

#define FLOPPY_IRQ 6

namespace Kernel {

class AsyncBlockDeviceRequest;
class FloppyDiskController;
class FloppyDiskDriveDevice;

struct PhysicalRegionDescriptor {
    PhysicalAddress offset;
    u16 size { 0 };
    u16 end_of_table { 0 };
};

class FloppyDiskDriveController final : public IRQHandler {
    friend class FloppyDiskController;
    friend class FloppyDiskDriveDevice;
    AK_MAKE_ETERNAL

public:
    static NonnullOwnPtr<FloppyDiskDriveController> create(const FloppyDiskController&, u8, bool);
    FloppyDiskDriveController(const FloppyDiskController&, u8, bool);
    virtual ~FloppyDiskDriveController() override;

    virtual const char* purpose() const override { return "82077AA controller"; }

    RefPtr<StorageDevice> device(u32 index) const;
    size_t devices_count() const { return m_devices.size(); }
    u8 label() const { return m_label; }
    char label_char() const { return '0' + m_label; } 

private:
    // valid only for primary controller
    enum class FloppyDiskDriveControllerRegisters: u16 {
        StatusRegisterA = 0x3f0,                // SRA  - read-only (PS/2 only)
        StatusRegisterB = 0x3f1,                // SRB  - read-only (PS/2 only)
        DigitalOutputRegister = 0x3f2,          // DOR
        TapeDriveRegister = 0x3f3,              // TDR
        MainStatusRegister = 0x3f4,             // MSR  - read-only
        DatarateSelectRegister = 0x3f4,         // DSR  - write-only (PS/2 only)
        DataFIFO = 0x3f5,                       // FIFO               
        DigitalInputRegister = 0x3f7,           // DIR  - read-only (AT only)
        ConfigurationControlRegister = 0x3f7    // CTRL - write-only (AT only)
    };

    enum class FloppyDiskDriveControllerCommands: u8 {
    	ReadTrack = 0x2,
	    Specify = 0x3,
	    CheckStatus = 0x4,
	    WriteSector = 0x5,
	    ReadSector = 0x6,
	    Calibrate = 0x7,
	    CheckIterrupt = 0x8,
	    WriteDeleteSector = 0x9,
	    ReadIdSector = 0xa,
	    ReadDeleteSector = 0xc,
	    FormatTrack = 0xd,
	    Seek = 0xf
    };

    //^ IRQHandler
    virtual void handle_irq(const RegisterState&) override;
    void expect_irq();
    void wait_for_irq();
    
    void read_sector_with_dma(u8, u32);
    void read_sector_with_polling(u8);

    void initialize();

    void detect_drives();
    char drive_label_char(u8 label) const { return 'a' + label; }
    String drive_type_string(u8) const;

    void initialize_dma();
    void setup_dma_read();
    void setup_dma_write();

    u8 read_status();
    void write_dor(u8);
    void write_ccr(u8);
    void send_cmd(u8);
    u8 read_data();

    void motor_control(u8, bool);
    
    bool calibrate(u8);

    void check_interrupt(u8&, u8&);

    void start_request(AsyncBlockDeviceRequest&, u8);
    void complete_current_request(AsyncDeviceRequest::RequestResult);

    void disable();
    void enable();

    void reset();

    void read_sector_imp(u8, u8, u8, u8);
    bool seek(u8, u8, u8);

    void lba_to_chs(u32, u8&, u8&, u8&);

    // Data members
    AsyncBlockDeviceRequest* m_current_request { nullptr };
    u32 m_current_request_block_index { 0 };
    SpinLock<u8> m_request_lock;

    NonnullRefPtr<FloppyDiskController> m_parent_controller;
    NonnullRefPtrVector<FloppyDiskDriveDevice> m_devices;
    u8 m_label;     // FDC0, FDC1 ...
    OwnPtr<Region> m_dma_region;
    bool m_use_dma { true };
    u16 irq_expected {0};
    WaitQueue m_irq_queue;
};

}