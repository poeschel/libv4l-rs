use std::convert::TryFrom;
use std::path::Path;
use std::sync::Arc;
use std::{io, mem};

use libc;

use crate::device::Handle;
use crate::Format;
use crate::v4l2;
use crate::v4l_sys::*;

/// Linux capture device abstraction
pub struct Subdevice {
    /// Raw handle
    handle: Arc<Handle>,
}

impl Subdevice {
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
        let path = format!("{}{}", "/dev/v4l-subdev", index);
        let fd = v4l2::open(&path, libc::O_RDWR | libc::O_NONBLOCK)?;

        if fd == -1 {
            return Err(io::Error::last_os_error());
        }

        Ok(Subdevice {
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
    /// let dev = Device::with_path("/dev/v4l-subdev0");
    /// ```
    pub fn with_path<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let fd = v4l2::open(&path, libc::O_RDWR | libc::O_NONBLOCK)?;

        if fd == -1 {
            return Err(io::Error::last_os_error());
        }

        Ok(Subdevice {
            handle: Arc::new(Handle::new(fd)),
        })
    }

    /// Returns the raw device handle
    pub fn handle(&self) -> Arc<Handle> {
        self.handle.clone()
    }


    pub fn format(&self) -> io::Result<Format> {
        unsafe {
            let mut v4l2_subdev_fmt = v4l2_subdev_format {
                ..mem::zeroed()
            };
            v4l2::ioctl(
                self.handle().fd(),
                v4l2::vidioc_subdev::VIDIOC_SUBDEV_G_FMT,
                &mut v4l2_subdev_fmt as *mut _ as *mut std::os::raw::c_void,
                )?;

            Ok(Format::from(v4l2_subdev_fmt.format))
        }
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

            println!("edid.blocks:{}", v4l2_edid.blocks);
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
}

