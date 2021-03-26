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

#define DEBUG_82077AA   1

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
    disable_irq();

    detect_drives();

    initialize();

    enable_irq();

}

UNMAP_AFTER_INIT FloppyDiskDriveController::~FloppyDiskDriveController()
{
}

void FloppyDiskDriveController::start_request(AsyncBlockDeviceRequest& request, u8 drive)
{
    if(request.request_type() == AsyncBlockDeviceRequest::Read){
        dbgln_if(DEBUG_82077AA, "fdc{:c}/fd{:c}: read from {} ({} blocks)", label_char(), drive_label_char(drive), request.block_index(), request.block_count());

      //  if(m_use_dma){
      //      read_sectors_with_dma(drive);
      //  } else {
            read_sectors_with_polling(request, drive);
      //  }

    } else {
        dbgln_if(DEBUG_82077AA, "fdc{:c}: write {} blocks to drive {:c}", label_char(), request.block_count(), drive_label_char(drive));
    }
}

void FloppyDiskDriveController::complete_current_request(AsyncDeviceRequest::RequestResult)
{
    TODO();
}

void FloppyDiskDriveController::handle_irq(const RegisterState&)
{
    TODO();
}

UNMAP_AFTER_INIT void FloppyDiskDriveController::detect_drives()
{
    // FIXME: Find better way to detect drives,
    // this checks only the main controller.
    IO::out8(0x70, 0x10);

    for (int i = 0; i < 4; ++i)
        IO::in8(0x71);

    u8 value_of_reg = IO::in8(0x71);

    u8 slave_nibble = value_of_reg & 0x0F;
    u8 master_nibble = (value_of_reg & 0xF0) >> 4;  // sould be 1.44 M, but got 2.88 M, bug ??

    // FIXME: Sort out usupported drives, check for size.
    if(master_nibble != FLOPPY_DRIVE_TYPE_NO_DRIVE){
        m_devices.append(FloppyDiskDriveDevice::create(m_parent_controller, *this, 0, 1440));
    }

    if(slave_nibble != FLOPPY_DRIVE_TYPE_NO_DRIVE){
        m_devices.append(FloppyDiskDriveDevice::create(m_parent_controller, *this, 1, 1440));
    }

    dbgln("FDC{}: master={}, slave={}", label_char(), drive_type_string(master_nibble), drive_type_string(slave_nibble));
}

UNMAP_AFTER_INIT void FloppyDiskDriveController::initialize(){
    if(m_use_dma){
        m_dma_buffer_page = MM.allocate_supervisor_physical_page();
    }
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

void FloppyDiskDriveController::read_sectors_with_dma(u8){
    TODO();
}

// FIXME: Dirty hack, just to see if it works
void FloppyDiskDriveController::read_sectors_with_polling(AsyncBlockDeviceRequest& request, u8 label){
    u8 zeros[512];
    (void) label;

    for(unsigned i = 0; i < 512; i++){
        zeros[i] = 0;
    }

    for(unsigned i = 0; i < request.block_count(); i++){e
        if (!request.write_to_buffer(request.buffer(), zeros, 512)) {
            request.complete(AsyncDeviceRequest::MemoryFault);
            return;
        }
    }

    request.complete(AsyncDeviceRequest::Success);
}

}