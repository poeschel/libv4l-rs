use bitflags::bitflags;
#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
use futures::ready;
#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
use futures::stream::Stream;
use std::convert::TryFrom;
use std::os::unix::io::{AsRawFd, RawFd};
#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
use std::pin::Pin;
use std::path::Path;
use std::sync::Arc;
#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
use std::task::{Context, Poll};
use std::{io, mem};

#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
use tokio::io::unix::{AsyncFd, TryIoError};
#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
use tokio::io::Interest;

use libc;

use crate::control;
use crate::format::FieldOrder;
use crate::v4l2;
use crate::v4l2::videodev::v4l2_ext_controls;
use crate::v4l_sys::*;
use crate::{capability::Capabilities, control::Control, Timestamp};

bitflags!{
    /// Event Request Flags
    ///
    /// Maps to kernel [`V4L2_EVENT_SUB_FL_*`] flags.
    pub struct EventRequestFlags: u32 {
        const NONE = 0;
        const SEND_INITIAL = (1 << 0);
        const ALLOW_FEEDBACK = (1 << 1);
    }
}

impl From<u32> for EventRequestFlags {
    fn from(flags: u32) -> Self {
        Self::from_bits_truncate(flags)
    }
}

impl From<EventRequestFlags> for u32 {
    fn from(flags: EventRequestFlags) -> Self {
        flags.bits()
    }
}

/// V4L2 Event Types
///
/// Maps to kernel [`V4L2_EVENT_*`] event types.
#[repr(u32)]
pub enum V4l2EventType {
    All = 0,
    Vsync = 1,
    Eos = 2,
    Ctrl = 3,
    FrameSync = 4,
    SourceChange = 5,
    MotionDet = 6,
    Unknown = 0xffffffff
}

impl From<u32> for V4l2EventType {
    fn from(val: u32) -> V4l2EventType {
        match val {
            0 => V4l2EventType::All,
            1 => V4l2EventType::Vsync,
            2 => V4l2EventType::Eos,
            3 => V4l2EventType::Ctrl,
            4 => V4l2EventType::FrameSync,
            5 => V4l2EventType::SourceChange,
            6 => V4l2EventType::MotionDet,
            _ => V4l2EventType::Unknown,
        }
    }
}

#[derive(Debug)]
#[repr(u32)]
pub enum CtrlEventChanges {
    Value = (1 << 0),
    Flags = (1 << 1),
    Range = (1 << 2),
    Unknown = 0xffffffff
}

impl From<u32> for CtrlEventChanges {
    fn from(val: u32) -> CtrlEventChanges {
        match val {
            1 => CtrlEventChanges::Value,
            2 => CtrlEventChanges::Flags,
            4 => CtrlEventChanges::Range,
            _ => CtrlEventChanges::Unknown
        }
    }
}

#[derive(Debug)]
pub struct CtrlEvent {
    pub changes: CtrlEventChanges,
    pub control: Control,
    pub flags: control::Flags,
    pub minimum: i32,
    pub maximum: i32,
    pub step: i32,
    pub default_value: i32
}

#[derive(Debug)]
#[repr(u32)]
pub enum SrcEvent {
    ChResolution = 1,
    Unknown = 0xffffffff
}

impl From<u32> for SrcEvent {
    fn from(val: u32) -> SrcEvent {
        match val {
            1 => SrcEvent::ChResolution,
            _ => SrcEvent::Unknown
        }
    }
}

#[derive(Debug)]
#[repr(u32)]
pub enum MotionDetEventFlag {
    HaveFrameSeq = 1,
    Unknown = 0xffffffff
}

impl From<u32> for MotionDetEventFlag {
    fn from(val: u32) -> MotionDetEventFlag {
        match val {
            1 => MotionDetEventFlag::HaveFrameSeq,
            _ => MotionDetEventFlag::Unknown
        }
    }
}

#[derive(Debug)]
pub struct MotionDetEvent {
    pub flags: MotionDetEventFlag,
    pub frame_sequence: u32,
    pub region_mask: u32
}

#[derive(Debug)]
pub enum V4l2Event {
    Vsync(FieldOrder),
    Ctrl(CtrlEvent),
    FrameSync(u32),
    SourceChange(SrcEvent),
    MotionDet(MotionDetEvent)
}

impl From<v4l2_event_vsync> for V4l2Event {
    fn from(event: v4l2_event_vsync) -> V4l2Event {
        V4l2Event::Vsync(FieldOrder::try_from(event.field as u32).unwrap_or(FieldOrder::Unknown))
    }
}

impl From<v4l2_event_ctrl> for V4l2Event {
    fn from(event: v4l2_event_ctrl) -> V4l2Event {
        V4l2Event::Ctrl(CtrlEvent {
            changes: CtrlEventChanges::from(event.changes),
            control: Control::from(event),
            flags: control::Flags::from(event.flags),
            minimum: event.minimum,
            maximum: event.maximum,
            step: event.step,
            default_value: event.default_value

        })
    }
}

impl From<v4l2_event_frame_sync> for V4l2Event {
    fn from(event: v4l2_event_frame_sync) -> V4l2Event {
        V4l2Event::FrameSync(event.frame_sequence)
    }
}

impl From<v4l2_event_src_change> for V4l2Event {
    fn from(event: v4l2_event_src_change) -> V4l2Event {
        V4l2Event::SourceChange(SrcEvent::from(event.changes))
    }
}

impl From<v4l2_event_motion_det> for V4l2Event {
    fn from(event: v4l2_event_motion_det) -> V4l2Event {
        V4l2Event::MotionDet(MotionDetEvent{
            flags: MotionDetEventFlag::from(event.flags),
            frame_sequence: event.frame_sequence,
            region_mask: event.region_mask
        })
    }
}

impl From<v4l2_event> for V4l2Event {
    fn from(event: v4l2_event) -> V4l2Event {
        unsafe {
            match V4l2EventType::from(event.type_) {
                V4l2EventType::Vsync => V4l2Event::from(event.u.vsync),
                V4l2EventType::Ctrl => V4l2Event::from(event.u.ctrl),
                V4l2EventType::FrameSync => V4l2Event::from(event.u.frame_sync),
                V4l2EventType::SourceChange => V4l2Event::from(event.u.src_change),
                V4l2EventType::MotionDet => V4l2Event::from(event.u.motion_det),
                _ => panic!()
            }
        }
    }
}

#[derive(Debug)]
pub struct V4l2BaseEvent {
    pub event: V4l2Event,
    pub pending: u32,
    pub sequence: u32,
    pub timestamp: Timestamp,
    pub id: u32

}

pub struct V4l2EventHandle {
    handle: Arc<Handle>
}

impl V4l2EventHandle {
    pub fn get_event(&mut self) -> io::Result<V4l2BaseEvent> {
        println!("get_event vor poll");
        let _res = self.handle.poll(libc::POLLPRI, -1)?;
        println!("nach poll");
        unsafe {
            let mut v4l2_event: v4l2_event = mem::zeroed();
            v4l2::ioctl(
                self.handle.fd,
                v4l2::vidioc::VIDIOC_DQEVENT,
                &mut v4l2_event as *mut _ as *mut std::os::raw::c_void,
            )?;

            let pending = v4l2_event.pending;
            let sequence = v4l2_event.sequence;
            let timestamp = Timestamp::from(v4l2_event.timestamp);
            let id = v4l2_event.id;
            let event = V4l2Event::from(v4l2_event);
            Ok(V4l2BaseEvent {
                event,
                pending,
                sequence,
                timestamp,
                id
            })
        }
    }
}

impl AsRawFd for V4l2EventHandle {
    fn as_raw_fd(&self) -> RawFd {
        println!("V4l2EventHandle::as_raw_fd()");
        self.handle.fd
    }
}

impl Iterator for V4l2EventHandle {
    type Item = io::Result<V4l2BaseEvent>;

    fn next(&mut self) -> Option<io::Result<V4l2BaseEvent>> {
        println!("V4l2EventHandle::next()");
        match self.get_event() {
            Ok(event) => Some(Ok(event)),
            Err(e) => Some(Err(e))
        }
    }
}

#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
pub struct AsyncV4l2EventHandle {
    asyncfd: AsyncFd<V4l2EventHandle>
}

#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
impl AsyncV4l2EventHandle {
    pub fn new(handle: V4l2EventHandle) -> io::Result<AsyncV4l2EventHandle> {
        Ok(AsyncV4l2EventHandle{
            asyncfd: AsyncFd::with_interest(handle, Interest::PRIORITY)?
        })
    }
}

#[cfg(feature = "async-tokio")]
#[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
impl Stream for AsyncV4l2EventHandle {
    type Item = io::Result<V4l2BaseEvent>;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context) -> Poll<Option<Self::Item>> {
        loop {
            let mut guard = ready!(self.asyncfd.poll_priority_ready_mut(cx))?;
            let res = guard.try_io(|inner| inner.get_mut().get_event());
            guard.clear_ready();
            match res {
                Err(TryIoError { .. }) => {
                    continue;
                },
                Ok(Ok(event)) => {
                    return Poll::Ready(Some(Ok(event)));
                },
                Ok(Err(err)) => {
                    return Poll::Ready(Some(Err(err.into())));
                }
            }
        }
    }
}

/// Linux capture device abstraction
pub struct Device {
    /// Raw handle
    handle: Arc<Handle>,
}

impl Device {
    /// Returns a capture device by index
    ///
    /// Devices are usually enumerated by the system.
    /// An index of zero thus represents the first device the system got to know about.
    ///
    /// # Arguments
    ///
    /// * `index` - Index (0: first, 1: second, ..)
    ///
    /// # Example
    ///
    /// ```
    /// use v4l::device::Device;
    /// let dev = Device::new(0);
    /// ```
    pub fn new(index: usize) -> io::Result<Self> {
        let path = format!("{}{}", "/dev/video", index);
        let fd = v4l2::open(&path, libc::O_RDWR | libc::O_NONBLOCK)?;

        if fd == -1 {
            return Err(io::Error::last_os_error());
        }

        Ok(Device {
            handle: Arc::new(Handle::new(fd)),
        })
    }

    /// Returns a capture device by path
    ///
    /// Linux device nodes are usually found in /dev/videoX or /sys/class/video4linux/videoX.
    ///
    /// # Arguments
    ///
    /// * `path` - Path (e.g. "/dev/video0")
    ///
    /// # Example
    ///
    /// ```
    /// use v4l::device::Device;
    /// let dev = Device::with_path("/dev/video0");
    /// ```
    pub fn with_path<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let fd = v4l2::open(&path, libc::O_RDWR | libc::O_NONBLOCK)?;

        if fd == -1 {
            return Err(io::Error::last_os_error());
        }

        Ok(Device {
            handle: Arc::new(Handle::new(fd)),
        })
    }

    /// Returns the raw device handle
    pub fn handle(&self) -> Arc<Handle> {
        self.handle.clone()
    }

    /// Returns video4linux framework defined information such as card, driver, etc.
    pub fn query_caps(&self) -> io::Result<Capabilities> {
        unsafe {
            let mut v4l2_caps: v4l2_capability = mem::zeroed();
            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_QUERYCAP,
                &mut v4l2_caps as *mut _ as *mut std::os::raw::c_void,
            )?;

            Ok(Capabilities::from(v4l2_caps))
        }
    }

    /// Returns the supported controls for a device such as gain, focus, white balance, etc.
    pub fn query_controls(&self) -> io::Result<Vec<control::Description>> {
        let mut controls = Vec::new();
        unsafe {
            let mut v4l2_ctrl: v4l2_query_ext_ctrl = mem::zeroed();

            loop {
                v4l2_ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
                v4l2_ctrl.id |= V4L2_CTRL_FLAG_NEXT_COMPOUND;
                match v4l2::ioctl(
                    self.handle().fd(),
                    v4l2::vidioc::VIDIOC_QUERY_EXT_CTRL,
                    &mut v4l2_ctrl as *mut _ as *mut std::os::raw::c_void,
                ) {
                    Ok(_) => {
                        // get the basic control information
                        let mut control = control::Description::from(v4l2_ctrl);

                        // if this is a menu control, enumerate its items
                        if control.typ == control::Type::Menu
                            || control.typ == control::Type::IntegerMenu
                        {
                            let mut items = Vec::new();

                            for i in (v4l2_ctrl.minimum..=v4l2_ctrl.maximum)
                                .step_by(v4l2_ctrl.step as usize)
                            {
                                let mut v4l2_menu = v4l2_querymenu {
                                    id: v4l2_ctrl.id,
                                    index: i as u32,
                                    ..mem::zeroed()
                                };
                                let res = v4l2::ioctl(
                                    self.handle().fd(),
                                    v4l2::vidioc::VIDIOC_QUERYMENU,
                                    &mut v4l2_menu as *mut _ as *mut std::os::raw::c_void,
                                );

                                // BEWARE OF DRAGONS!
                                // The API docs [1] state VIDIOC_QUERYMENU should may return EINVAL
                                // for some indices between minimum and maximum when an item is not
                                // supported by a driver.
                                //
                                // I have no idea why it is advertised in the first place then, but
                                // have seen this happen with a Logitech C920 HD Pro webcam.
                                // In case of errors, let's just skip the offending index.
                                //
                                // [1] https://github.com/torvalds/linux/blob/master/Documentation/userspace-api/media/v4l/vidioc-queryctrl.rst#description
                                if res.is_err() {
                                    continue;
                                }

                                let item =
                                    control::MenuItem::try_from((control.typ, v4l2_menu)).unwrap();
                                items.push((v4l2_menu.index, item));
                            }

                            control.items = Some(items);
                        }

                        controls.push(control);
                    }
                    Err(e) => {
                        if controls.is_empty() || e.kind() != io::ErrorKind::InvalidInput {
                            return Err(e);
                        } else {
                            break;
                        }
                    }
                }
            }
        }

        Ok(controls)
    }

    /// Returns the edid blocks, if your device has edid
    pub fn query_edid(&self) -> io::Result<Vec<u8>> {
        unsafe {
            let mut v4l2_edid: v4l2_edid = mem::zeroed();
            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_G_EDID,
                &mut v4l2_edid as *mut _ as *mut std::os::raw::c_void,
            )?;

            let blocks = match usize::try_from(v4l2_edid.blocks * 128) {
                Ok(b) => b,
                Err(_e) => {
                    return Err(io::Error::new(
                            io::ErrorKind::InvalidInput,
                            "You seem to have waaaaay to many edid blocks.",
                            ));
                }
            };

            let mut vec = Vec::with_capacity(blocks);
            v4l2_edid.edid = vec.as_mut_ptr();
            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_G_EDID,
                &mut v4l2_edid as *mut _ as *mut std::os::raw::c_void,
            )?;

            Ok(vec)
        }
    }

    /// Returns the control value for an ID
    ///
    /// # Arguments
    ///
    /// * `id` - Control identifier
    pub fn control(&self, id: u32) -> io::Result<Control> {
        unsafe {
            let mut queryctrl = v4l2_query_ext_ctrl {
                id,
                ..mem::zeroed()
            };
            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_QUERY_EXT_CTRL,
                &mut queryctrl as *mut _ as *mut std::os::raw::c_void,
            )?;

            // determine the control type
            let description = control::Description::from(queryctrl);

            // query the actual control value
            let mut v4l2_ctrl = v4l2_ext_control {
                id,
                ..mem::zeroed()
            };
            let mut v4l2_ctrls = v4l2_ext_controls {
                count: 1,
                controls: &mut v4l2_ctrl,
                ..mem::zeroed()
            };
            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_G_EXT_CTRLS,
                &mut v4l2_ctrls as *mut _ as *mut std::os::raw::c_void,
            )?;

            let value = match description.typ {
                control::Type::Integer | control::Type::Integer64 | control::Type::Menu => {
                    control::Value::Integer(v4l2_ctrl.__bindgen_anon_1.value64)
                }
                control::Type::Boolean => {
                    control::Value::Boolean(v4l2_ctrl.__bindgen_anon_1.value64 == 1)
                }
                _ => {
                    return Err(io::Error::new(
                        io::ErrorKind::Other,
                        "cannot handle control type",
                    ))
                }
            };

            Ok(Control { id, value })
        }
    }

    /// Modifies the control value
    ///
    /// # Arguments
    ///
    /// * `ctrl` - Control to be set
    pub fn set_control(&self, ctrl: Control) -> io::Result<()> {
        self.set_controls(vec![ctrl])
    }

    /// Modifies the control values atomically
    ///
    /// # Arguments
    ///
    /// * `ctrls` - Vec of the controls to be set
    pub fn set_controls(&self, ctrls: Vec<Control>) -> io::Result<()> {
        unsafe {
            let mut control_list: Vec<v4l2_ext_control> = vec![];
            let mut class: Option<u32> = None;

            if ctrls.is_empty() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "ctrls cannot be empty",
                ));
            }

            for ref ctrl in ctrls {
                let mut control = v4l2_ext_control {
                    id: ctrl.id,
                    ..mem::zeroed()
                };
                class = match class {
                    Some(c) => {
                        if c != (control.id & 0xFFFF0000) {
                            return Err(io::Error::new(
                                io::ErrorKind::InvalidInput,
                                "All controls must be in the same class",
                            ));
                        } else {
                            Some(c)
                        }
                    }
                    None => Some(control.id & 0xFFFF0000),
                };

                match ctrl.value {
                    control::Value::None => {}
                    control::Value::Integer(val) => {
                        control.__bindgen_anon_1.value64 = val;
                        control.size = std::mem::size_of::<i64>() as u32;
                    }
                    control::Value::Boolean(val) => {
                        control.__bindgen_anon_1.value64 = val as i64;
                        control.size = std::mem::size_of::<i64>() as u32;
                    }
                    control::Value::String(ref val) => {
                        control.__bindgen_anon_1.string = val.as_ptr() as *mut std::os::raw::c_char;
                        control.size = val.len() as u32;
                    }
                    control::Value::CompoundU8(ref val) => {
                        control.__bindgen_anon_1.p_u8 = val.as_ptr() as *mut u8;
                        control.size = (val.len() * std::mem::size_of::<u8>()) as u32;
                    }
                    control::Value::CompoundU16(ref val) => {
                        control.__bindgen_anon_1.p_u16 = val.as_ptr() as *mut u16;
                        control.size = (val.len() * std::mem::size_of::<u16>()) as u32;
                    }
                    control::Value::CompoundU32(ref val) => {
                        control.__bindgen_anon_1.p_u32 = val.as_ptr() as *mut u32;
                        control.size = (val.len() * std::mem::size_of::<u32>()) as u32;
                    }
                    control::Value::CompoundPtr(ref val) => {
                        control.__bindgen_anon_1.ptr = val.as_ptr() as *mut std::os::raw::c_void;
                        control.size = (val.len() * std::mem::size_of::<u8>()) as u32;
                    }
                };

                control_list.push(control);
            }

            let class = class.ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "failed to determine control class",
                )
            })?;

            let mut controls = v4l2_ext_controls {
                count: control_list.len() as u32,
                controls: control_list.as_mut_ptr(),

                which: class,
                ..mem::zeroed()
            };

            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_S_EXT_CTRLS,
                &mut controls as *mut _ as *mut std::os::raw::c_void,
            )
        }
    }

    pub fn set_edid(&self, edid: &mut [u8]) -> io::Result<()> {
        let len = edid.len();
        if len % 128 != 0 {
            return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "The provided edid data has invalid length. One edid block is 128 bytes. Your edid block is not divisible by 128.",
                    ));
        }

        let blocks = match u32::try_from(len / 128) {
            Ok(b) => b,
            Err(_e) => {
            return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "You seem to have waaaaay to many edid blocks.",
                    ));
            }
        };

        unsafe {
            let mut v4l2_edid: v4l2_edid = mem::zeroed();
            v4l2_edid.blocks = blocks;
            v4l2_edid.edid = edid.as_mut_ptr();

            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_S_EDID,
                &mut v4l2_edid as *mut _ as *mut std::os::raw::c_void,
            )
        }
    }

    pub fn events(&self, r#type: V4l2EventType, flags: EventRequestFlags) -> io::Result<V4l2EventHandle> {
        unsafe {
            let mut sub = v4l2_event_subscription {
                type_: r#type as u32,
                id: 0,
                flags: u32::from(flags),
                reserved: mem::zeroed()
            };

            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc::VIDIOC_SUBSCRIBE_EVENT,
                &mut sub as *mut _ as *mut std::os::raw::c_void,
            )?;

            Ok(V4l2EventHandle {
                handle: self.handle.clone()
            })
        }

    }


    #[cfg(feature = "async-tokio")]
    #[cfg_attr(docsrs, doc(cfg(feature = "async-tokio")))]
    pub fn async_events(&self, r#type: V4l2EventType, flags: EventRequestFlags) -> io::Result<AsyncV4l2EventHandle> {
        let events = self.events(r#type, flags)?;
        Ok(AsyncV4l2EventHandle::new(events)?)
    }
}

impl io::Read for Device {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        unsafe {
            let ret = libc::read(
                self.handle().fd(),
                buf.as_mut_ptr() as *mut std::os::raw::c_void,
                buf.len(),
            );
            match ret {
                -1 => Err(io::Error::last_os_error()),
                ret => Ok(ret as usize),
            }
        }
    }
}

impl io::Write for Device {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        unsafe {
            let ret = libc::write(
                self.handle().fd(),
                buf.as_ptr() as *const std::os::raw::c_void,
                buf.len(),
            );

            match ret {
                -1 => Err(io::Error::last_os_error()),
                ret => Ok(ret as usize),
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        // write doesn't use a buffer, so it effectively flushes with each call
        // therefore, we don't have anything to flush later
        Ok(())
    }
}

/// Device handle for low-level access.
///
/// Acquiring a handle facilitates (possibly mutating) interactions with the device.
pub struct Handle {
    fd: std::os::raw::c_int,
}

impl Handle {
    pub (crate) fn new(fd: std::os::raw::c_int) -> Self {
        Self { fd }
    }

    /// Returns the raw file descriptor
    pub fn fd(&self) -> std::os::raw::c_int {
        self.fd
    }

    /// Polls the file descriptor for I/O events
    ///
    /// # Arguments
    ///
    /// * `events`  - The events you are interested in (e.g. POLLIN)
    ///
    /// * `timeout` - Timeout in milliseconds
    ///               A value of zero returns immedately, even if the fd is not ready.
    ///               A negative value means infinite timeout (blocking).
    pub fn poll(&self, events: i16, timeout: i32) -> io::Result<i32> {
        match unsafe {
            libc::poll(
                [libc::pollfd {
                    fd: self.fd,
                    events,
                    revents: 0,
                }]
                .as_mut_ptr(),
                1,
                timeout,
            )
        } {
            -1 => Err(io::Error::last_os_error()),
            ret => {
                // A return value of zero means that we timed out. A positive value signifies the
                // number of fds with non-zero revents fields (aka I/O activity).
                assert!(ret == 0 || ret == 1);
                Ok(ret)
            }
        }
    }
}

impl Drop for Handle {
    fn drop(&mut self) {
        v4l2::close(self.fd).unwrap();
    }
}

impl AsRawFd for Handle {
    fn as_raw_fd(&self) -> RawFd {
        self.fd
    }
}
