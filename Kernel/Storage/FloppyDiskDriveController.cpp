/*
 * Copyright (c) 2018-2021, Andreas Kling <kling@serenityos.org>
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

#include <AK/ByteBuffer.h>
#include <AK/Singleton.h>
#include <AK/StringView.h>
#include <Kernel/FileSystem/ProcFS.h>
#include <Kernel/IO.h>
#include <Kernel/Process.h>
#include <Kernel/Storage/FloppyDiskController.h>
#include <Kernel/Storage/FloppyDiskDriveController.h>
#include <Kernel/Storage/FloppyDiskDriveDevice.h>
#include <Kernel/VM/MemoryManager.h>
#include <Kernel/WorkQueue.h>
#include <Kernel/WaitQueue.h>
#include <Kernel/CMOS.h>
#include <Kernel/VM/AnonymousVMObject.h>

#define DEBUG_FDC   1

namespace Kernel {

UNMAP_AFTER_INIT NonnullOwnPtr<FloppyDiskDriveController> FloppyDiskDriveController::create(const FloppyDiskController& controller, u8 label, bool use_dma)
{
    return make<FloppyDiskDriveController>(controller, label, use_dma);
}

UNMAP_AFTER_INIT FloppyDiskDriveController::FloppyDiskDriveController(const FloppyDiskController& controller, u8 label,  bool use_dma)
    : IRQHandler(FLOPPY_IRQ)
    , m_parent_controller(controller)
    , m_label(label)
    , m_use_dma(use_dma)
{
    detect_drives();

    initialize();
}

UNMAP_AFTER_INIT FloppyDiskDriveController::~FloppyDiskDriveController()
{
}

void FloppyDiskDriveController::start_request(AsyncBlockDeviceRequest& request, u8 drive)
{
    m_current_request = &request;

    if(request.request_type() == AsyncBlockDeviceRequest::Read){
        dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: requested read from {} ({} blocks)", label_char(), drive_label_char(drive), request.block_index(), request.block_count());

        if(m_use_dma){
            read_sector_with_dma(drive, request.block_index());
        } else {
            read_sector_with_polling(drive);
        }
    } else {
        dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: requested write to {} ({} blocks)", label_char(), drive_label_char(drive), request.block_index(), request.block_count());
    }
}

void FloppyDiskDriveController::complete_current_request(AsyncDeviceRequest::RequestResult result)
{
        dbgln_if(DEBUG_FDC, "fdc{:c}: completed current request, result={}", label_char(), (int)result);
        VERIFY(m_current_request);
        auto& request = *m_current_request;

        if (result == AsyncDeviceRequest::Success) {
            if (request.request_type() == AsyncBlockDeviceRequest::Read) {
                if (!request.write_to_buffer(request.buffer(), m_dma_region->vaddr().as_ptr(), 512)) {                        
                    request.complete(AsyncDeviceRequest::MemoryFault);
                    goto cleanup;
                }

                dbgln_if(DEBUG_FDC, "fdc{:c}: succesfully read {} blocks", label_char(), request.block_count());
#if DEBUG_FDC
                u32 addr = (u32)m_dma_region->vaddr().as_ptr();
                //u32 addr = m_dma_region->physical_page(0)->paddr().get();
                const u8 * buf = (u8* )addr;
                dbgln("fdc{:c}: buffer={:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} ...", label_char(), buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
#endif
            }
        }

        request.complete(result);
cleanup:
        m_current_request = nullptr;
}

 void FloppyDiskDriveController::expect_irq(){
    irq_expected++;
    enable_irq();
 }

void FloppyDiskDriveController::wait_for_irq(){
    m_irq_queue.wait_forever("FDC");
    disable_irq();
}

void FloppyDiskDriveController::handle_irq(const RegisterState&)
{
    if(irq_expected == 0){
        dbgln("fdc{:c}: unexpected interrupt.", label_char());
    } else {
        dbgln_if(DEBUG_FDC, "fdc{:c}: interrupt.", label_char());
        irq_expected--;
    }

    m_irq_queue.wake_all();
}

UNMAP_AFTER_INIT void FloppyDiskDriveController::detect_drives()
{
    // FIXME: Find better way to detect drives,
    // this checks only the main controller.
    u8 value_of_reg = CMOS::read(0x10);

    u8 slave_nibble = value_of_reg & 0x0F;
    u8 master_nibble = (value_of_reg & 0xF0) >> 4;  // sould be 1.44 M, but got 2.88 M, bug ??

    // FIXME: Sort out usupported drives, check for size.
    if(master_nibble != FLOPPY_DRIVE_TYPE_NO_DRIVE){
        m_devices.append(FloppyDiskDriveDevice::create(m_parent_controller, *this, 0, 1440));
    }

    if(slave_nibble != FLOPPY_DRIVE_TYPE_NO_DRIVE){
        m_devices.append(FloppyDiskDriveDevice::create(m_parent_controller, *this, 1, 1440));
    }

    dbgln("fdc{:c}: master={} slave={}", label_char(), drive_type_string(master_nibble), drive_type_string(slave_nibble));
}

UNMAP_AFTER_INIT void FloppyDiskDriveController::initialize(){
    u8 res = 0;

    if(m_use_dma){
        initialize_dma();
    } else {
        TODO();
    }

    send_cmd(FDC_CMD_CONFIGURE);
    send_cmd(0);
    send_cmd((1 << 6) | (0 << 5) | (1 << 4) | 8);
    send_cmd(0);

    // if floppy is 2.88 M
    send_cmd(FDC_CMD_PERPENDICULAR);
    send_cmd(1 << 2);

    send_cmd(FDC_CMD_LOCK);
    res = read_data();
    if(res != 0x10){
        VERIFY_NOT_REACHED();
    }

    reset();

    for(u8 i = 0; i < (u8)m_devices.size(); i++){
        calibrate(i);
    }
}

void FloppyDiskDriveController::initialize_dma(){
    auto page = MM.allocate_supervisor_physical_page();
    VERIFY(page);
    auto vmobject = AnonymousVMObject::create_with_physical_page(*page);
    m_dma_region = MM.allocate_kernel_region_with_vmobject(*vmobject, PAGE_SIZE, String::format("fd%c DMA buffer", label_char()), Region::Access::Read | Region::Access::Write);
    VERIFY(m_dma_region);

    u32 addr = (u32)m_dma_region->physical_page(0)->paddr().get();
    u16 count = 512 - 1;   // -1 because of DMA counting

    VERIFY(count <= PAGE_SIZE);

    u8 addr_0 = (addr & 0x000000ff);
    u8 addr_1 = (addr & 0x0000ff00) >> 8;
    u8 addr_2 = (addr & 0x00ff0000) >> 16;

    u8 count_0 = count & 0x00ff;
    u8 count_1 = (count & 0xff00) >> 8;

    dbgln_if(DEBUG_FDC, "fdc{:c} DMA address={:08x} count={:04x}({})", label_char(), addr, count, count);

    IO::out8(0x0a, 0x06);       // mask dma channel 2
    IO::out8(0x0c, 0xff);	    // reset master flip-flop
    IO::out8(0x04, addr_0);     // dma address
	IO::out8(0x04, addr_1);
	IO::out8(0x0c, 0xff);	    // reset master flip-flop
	IO::out8(0x05, count_0);    // dma count
	IO::out8(0x05, count_1);
    IO::out8(0x81, addr_2);     // external page register 
	IO::out8(0x0a, 0x02);       // unmask dma channel 2

}

void FloppyDiskDriveController::setup_dma_read(){
    IO::out8(0x0a, 0x06); // mask dma channel 2
	IO::out8(0x0b, 0x56); // single transfer, address increment, autoinit, read, channel 2
	IO::out8(0x0a, 0x02); // unmask dma channel 2
}

void FloppyDiskDriveController::setup_dma_write(){
    IO::out8(0x0a, 0x06); // mask dma channel 2
	IO::out8(0x0b, 0x5a); // single transfer, address increment, autoinit, write, channel 2
	IO::out8(0x0a, 0x02); // unmask dma channel 2
}

u8 FloppyDiskDriveController::read_status(){
    return IO::in8(static_cast<u16>(FloppyDiskDriveControllerRegisters::MainStatusRegister));
}

void FloppyDiskDriveController::write_dor(u8 value){
    IO::out8(static_cast<u16>(FloppyDiskDriveControllerRegisters::DigitalOutputRegister), value);
}

void FloppyDiskDriveController::write_ccr(u8 value){
    IO::out8(static_cast<u16>(FloppyDiskDriveControllerRegisters::ConfigurationControlRegister), value);
}

void FloppyDiskDriveController::send_cmd(u8 command){
	//! wait until data register is ready. We send commands to the data register
	for (u16 i = 0; i < 500; i++ ){
		if (read_status() & FDC_MSR_MASK_DATAREG){
            IO::out8(static_cast<u16>(FloppyDiskDriveControllerRegisters::DataFIFO), command);
            return;
        }
    }
}

u8 FloppyDiskDriveController::read_data(void){
	//! same as above function but returns data register for reading
	for (u16 i = 0; i < 500; i++ ){
        if (read_status() & FDC_MSR_MASK_DATAREG){
            return IO::in8(static_cast<u16>(FloppyDiskDriveControllerRegisters::DataFIFO));
        }
    }
		
	return 0;
}

void FloppyDiskDriveController::motor_control(u8 label, bool on){
    u8 motor = 0;

    switch (label) {
		case 0:
			motor = FDC_DOR_MASK_DRIVE0_MOTOR;
			break;
		case 1:
			motor = FDC_DOR_MASK_DRIVE1_MOTOR;
			break;
		case 2:
			motor = FDC_DOR_MASK_DRIVE2_MOTOR;
			break;
		case 3:
			motor = FDC_DOR_MASK_DRIVE3_MOTOR;
			break;
        default:
            VERIFY_NOT_REACHED();
	}

    if (on){
        write_dor(label | motor | FDC_DOR_MASK_RESET | FDC_DOR_MASK_DMA);
    } else {
        write_dor(FDC_DOR_MASK_RESET);
    }

    // FIXME: wait for motor to spin up
    IO::delay(300);
}

bool FloppyDiskDriveController::calibrate(u8 label){
    u8 st0, cyl;

	//! turn on the motor
	motor_control(label, true);

	for (u8 i = 0; i < 10; i++) {
        expect_irq();

		//! send command
		send_cmd(FDC_CMD_CALIBRATE);
		send_cmd(label);
		
        wait_for_irq();
		
        check_interrupt(st0, cyl);

		//! did we fine cylinder 0? if so, we are done
		if (cyl == 0) {
			motor_control(label, false);

            dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: calibrated.", label_char(), drive_label_char(label));

			return true;
		}
	}

	motor_control(label, false);

    VERIFY_NOT_REACHED();

	return false;
}

void FloppyDiskDriveController::check_interrupt(u8& st0, u8& cyl){
    send_cmd(FDC_CMD_check_interrupt);

    st0 = read_data();
	cyl = read_data();
}

void FloppyDiskDriveController::disable(){
    write_dor(0x00);
}

void FloppyDiskDriveController::enable(){
    //write_dor(FDC_DOR_MASK_RESET | FDC_DOR_MASK_DMA);
    write_dor(0x0c);
}

void FloppyDiskDriveController::reset(){
    expect_irq();

    disable();
	enable();

    wait_for_irq();
    // sense interrupt -- 4 of them typically required after a reset
    for (u8 i = 0; i < 4; i++){
        u8 st0, cyl; // ignore these here..
        check_interrupt(st0, cyl);
    }

    //! transfer speed 500kb/s
	// write_ccr(0x00);     // 1.44 M, 1.2 M
    write_ccr(0x03);        // 1.88 M

	//  - 1st byte is: bits[7:4] = steprate, bits[3:0] = head unload time
    //  - 2nd byte is: bits[7:1] = head load time, bit[0] = no-DMA
    //
    //  steprate    = (8.0ms - entry*0.5ms)*(1MB/s / xfer_rate)
    //  head_unload = 8ms * entry * (1MB/s / xfer_rate), where entry 0 -> 16
    //  head_load   = 1ms * entry * (1MB/s / xfer_rate), where entry 0 -> 128
    //
    send_cmd(FDC_CMD_SPECIFY);
    send_cmd(0xdf); /* steprate = 3ms, unload time = 240ms */
    send_cmd(0x02); /* load time = 16ms, no-DMA = 0 -> use dma */

    dbgln_if(DEBUG_FDC, "fdc{:c}: reseted.", label_char());
}

void FloppyDiskDriveController::read_sector_imp(u8 label, u8 head, u8 track, u8 sector){
    dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: reading from head={} track={} sector={}", label_char(), drive_label_char(label), head, track, sector);

	setup_dma_read();

	send_cmd(FDC_CMD_READ_SECT | FDC_CMD_EXT_MULTITRACK | FDC_CMD_EXT_DENSITY);
	send_cmd(head << 2 | label);
	send_cmd(track);
	send_cmd(head);
	send_cmd(sector);
	send_cmd(FDC_SECTOR_DTL_512);
	send_cmd((( sector + 1 ) >= FDC_SECTORS_PER_TRACK ) ? FDC_SECTORS_PER_TRACK : sector + 1 );
	send_cmd(FDC_GAP3_LENGTH_3_5 );
	send_cmd(0xFF);

    dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: read command sent.", label_char(), drive_label_char(label));
}

bool FloppyDiskDriveController::seek(u8 label, u8 cyl, u8 head){
    u8 st0, cyl0;

    dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: seeking: cyl={} head={}", label_char(), drive_label_char(label), cyl, head);

    motor_control(label, true);

    for (u8 i = 0; i < 10; i++ ) {
        expect_irq();

		send_cmd(FDC_CMD_SEEK);
		send_cmd((head << 2) | label);
		send_cmd(cyl);

		wait_for_irq();   // seek does not fires an interrupt, but it should ??

		check_interrupt(st0, cyl0);

        if(st0 & 0xC0) {
            static const char * status[] = { "normal", "error", "invalid", "drive" };
            dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: seek status={}\n", label_char(), drive_label_char(label), status[st0 >> 6]);
        }

		if (cyl0 == cyl){
            dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: seeking done.", label_char(), drive_label_char(label));
            return true;
        }
	}

    motor_control(label, false);

    dbgln_if(DEBUG_FDC, "fdc{:c}/fd{:c}: seek error!", label_char(), drive_label_char(label));
	return false;
}

void FloppyDiskDriveController::lba_to_chs(u32 lba, u8& head, u8& track, u8& sector){
    head = (lba % (FDC_SECTORS_PER_TRACK * 2)) / (FDC_SECTORS_PER_TRACK);
	track = lba / (FDC_SECTORS_PER_TRACK * 2);
	sector = lba % FDC_SECTORS_PER_TRACK + 1;
}

String FloppyDiskDriveController::drive_type_string(u8 drive_type) const{
    switch(drive_type){
    case FLOPPY_DRIVE_TYPE_NO_DRIVE: return "no drive";
    case FLOPPY_DRIVE_TYPE_360KB_525IN: return "360 kB 5.25\"";
    case FLOPPY_DRIVE_TYPE_1200KB_525IN: return "1.2 MB 5.25\"";
    case FLOPPY_DRIVE_TYPE_720KB_35IN: return "720 kB 3.5\"";
    case FLOPPY_DRIVE_TYPE_1440KB_35IN : return "1.44 MB 3.5\"";
    case FLOPPY_DRIVE_TYPE_2880KB_35IN: return "2.88 MB 3.5\"";
    default:
        VERIFY_NOT_REACHED();
    }
}

RefPtr<StorageDevice> FloppyDiskDriveController::device(u32 index) const{
    VERIFY(index < m_devices.size());
    return m_devices[index];
}

void FloppyDiskDriveController::read_sector_with_dma(u8 label, u32 block){
    u8 st0, cyl;
    u8 head = 0, track = 0, sector = 0;

	lba_to_chs(block, head, track, sector);
    motor_control(label, true);

    if(seek (label, track, head) == false){
        VERIFY_NOT_REACHED();
    }

    expect_irq();

    read_sector_imp(label, head, track, sector);

    wait_for_irq();

    motor_control(label, false);

    // FIXME: reason this
    for (u8 i = 0; i < 7; i++)
	 	read_data();
	    
	check_interrupt(st0, cyl);

    // FIXME: check status
    complete_current_request(AsyncDeviceRequest::Success);
}

void FloppyDiskDriveController::read_sector_with_polling(u8){
    TODO();
}

}