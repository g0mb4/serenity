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

#include <AK/Memory.h>
#include <AK/StringView.h>
#include <Kernel/FileSystem/FileDescription.h>
#include <Kernel/Storage/FloppyDiskDriveController.h>
#include <Kernel/Storage/FloppyDiskController.h>
#include <Kernel/Storage/FloppyDiskDriveDevice.h>

namespace Kernel {

UNMAP_AFTER_INIT NonnullRefPtr<FloppyDiskDriveDevice> FloppyDiskDriveDevice::create(const FloppyDiskController& controller, FloppyDiskDriveController& drive_controller, u8 label, u64 max_addressable_block)
{
    return adopt(*new FloppyDiskDriveDevice(controller, drive_controller, label, max_addressable_block));
}

UNMAP_AFTER_INIT FloppyDiskDriveDevice::FloppyDiskDriveDevice(const FloppyDiskController& controller, FloppyDiskDriveController& drive_controller, u8 label, u64 max_addressable_block)
    : StorageDevice(controller, 512, max_addressable_block)
    , m_drive_controller(drive_controller)
    , m_label(label)
{
}

UNMAP_AFTER_INIT FloppyDiskDriveDevice::~FloppyDiskDriveDevice()
{
}

const char* FloppyDiskDriveDevice::class_name() const
{
    return "FloppyDiskDriveDevice";
}

void FloppyDiskDriveDevice::start_request(AsyncBlockDeviceRequest& request)
{
    m_drive_controller.start_request(request, m_label);
}

String FloppyDiskDriveDevice::device_name() const
{
    // FIXME: Do continous labeling.
    // One controller can handle 4 drives.
    char name = 'a' + (m_drive_controller.label() * 4 + m_label);
    return String::formatted("fd{:c}", name);
}

}
