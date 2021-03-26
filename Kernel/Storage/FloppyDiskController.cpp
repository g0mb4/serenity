/*
 * Copyright (c) 2020, Liav A. <liavalb@hotmail.co.il>
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

#include <AK/OwnPtr.h>
#include <AK/RefPtr.h>
#include <AK/Types.h>
#include <Kernel/Storage/FloppyDiskController.h>
#include <Kernel/Storage/FloppyDiskDriveController.h>

namespace Kernel {

UNMAP_AFTER_INIT NonnullRefPtr<FloppyDiskController> FloppyDiskController::initialize()
{
    return adopt(*new FloppyDiskController());
}

bool FloppyDiskController::reset()
{
    TODO();
}

bool FloppyDiskController::shutdown()
{
    TODO();
}

void FloppyDiskController::start_request(const StorageDevice&, AsyncBlockDeviceRequest&)
{
    VERIFY_NOT_REACHED();
}

void FloppyDiskController::complete_current_request(AsyncDeviceRequest::RequestResult)
{
    VERIFY_NOT_REACHED();
}

UNMAP_AFTER_INIT FloppyDiskController::FloppyDiskController()
    : StorageController()
{
    detect_drives();
}

UNMAP_AFTER_INIT FloppyDiskController::~FloppyDiskController()
{
}

void FloppyDiskController::detect_drives(){
    // Asking the CMOS about the types of the connected 
    // floppy disk drives.
    IO::out8(0x70, 0x10);
    
    // Using some delay on reading resgister, so it
    // has some time to react.
    for (int i = 0; i < 4; ++i)
        IO::in8(0x71);

    u8 value_of_reg = IO::in8(0x71);

    // FIXME: Find better way to detect controllers,
    // this does not check the controller type and 
    // there can be multiple controllers as well.
    if(value_of_reg != 0){
        m_drive_controllers.append(FloppyDiskDriveController::create(*this));
    }
}

size_t FloppyDiskController::devices_count() const {
    return 0;
}

RefPtr<StorageDevice> FloppyDiskController::device(u32 index) const
{
    (void)index;
    return nullptr;
}

}
