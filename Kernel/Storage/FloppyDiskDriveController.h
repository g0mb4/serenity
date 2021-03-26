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

#define FLOPPY_IRQ 6

namespace Kernel {

class AsyncBlockDeviceRequest;
class FloppyDiskController;

class FloppyDiskDriveController final : public IRQHandler {
    friend class FloppyDiskController;
    AK_MAKE_ETERNAL

public:
    static NonnullOwnPtr<FloppyDiskDriveController> create(const FloppyDiskController&);
    FloppyDiskDriveController(const FloppyDiskController&);
    virtual ~FloppyDiskDriveController() override;

    virtual const char* purpose() const override { return "82077AA controller"; }

private:
    //^ IRQHandler
    virtual void handle_irq(const RegisterState&) override;

    void detect_drives();
    String drive_type_string(u8) const;

    void start_request(AsyncBlockDeviceRequest&, u8 label);
    void complete_current_request(AsyncDeviceRequest::RequestResult);

    // Data members

    NonnullRefPtr<FloppyDiskController> m_parent_controller;
};

}