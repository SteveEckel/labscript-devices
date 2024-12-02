#####################################################################
#                                                                   #
# /labscript_devices/SpinnakerCamera/blacs_workers.py               #
#                                                                   #
# Copyright 2019, Monash University and contributors                #
#                                                                   #
# This file is part of labscript_devices, in the labscript suite    #
# (see http://labscriptsuite.org), and is licensed under the        #
# Simplified BSD License. See the license.txt file in the root of   #
# the project for the full license.                                 #
#                                                                   #
#####################################################################

# Original imaqdx_camera server by dt, with modifications by rpanderson and cbillington.
# Refactored as a BLACS worker by cbillington
# PsSpin implementation by spe

import numpy as np
from labscript_utils import dedent
from enum import IntEnum
from time import sleep, perf_counter

from labscript_devices.IMAQdxCamera.blacs_workers import IMAQdxCameraWorker

import h5py
import labscript_utils.properties
import threading

class Spinnaker_Camera(object):
    def __init__(self, serial_number):
        """Initialize Spinnaker API camera.

        Serial number should be of string(?) type."""
        global PySpin
        import PySpin

        self.system = PySpin.System.GetInstance()

        ver = self.system.GetLibraryVersion()
        min_ver = (1,23,0,27) # first release with python 3.6 support
        if (ver.major, ver.minor, ver.type, ver.build) < min_ver:
            raise RuntimeError(f"PySpin version {ver} must be >= {min_ver}")

        camList = self.system.GetCameras()
        numCams = camList.GetSize()

        if numCams==0:
            raise ValueError('No cameras found!')

        if isinstance(serial_number, int):
            self.camera = camList.GetBySerial('%d' % serial_number)
        else:
            self.camera = camList.GetBySerial(serial_number)
        self.camera.Init()
        camList.Clear()

        # Set the default image timeout to 1000 ms.  This is how long we wait
        # for a new image.  When we are grabbing multiple images (in buffered
        # mode), this timeout can be added to the time between exposures
        self.timeout = 1000 # in ms

        # Set the abort acquisition thingy:
        self._abort_acquisition = False
        self.exception_on_failed_shot = True

    def get_attribute_names(self, visibility):
        names = []
        def get_node_names_in_category(node_category, prefix=''):
            for node_feature in node_category.GetFeatures():
                # Ensure node is available and readable
                if (not PySpin.IsAvailable(node_feature) or not
                    PySpin.IsReadable(node_feature)):
                    continue

                # Get the feature name:
                feature_name = node_feature.GetName()

                # Category nodes must be dealt with separately in order to retrieve subnodes recursively.
                if node_feature.GetPrincipalInterfaceType() == PySpin.intfICategory:
                    get_node_names_in_category(PySpin.CCategoryPtr(node_feature),
                                               prefix=feature_name + '::')
                else:
                    names.append(prefix + feature_name)

        node = self.camera.GetNodeMap()
        get_node_names_in_category(PySpin.CCategoryPtr(node.GetNode('Root')))

        return names

    def get_attribute(self, name, stream_map=False):
        """Return current values dictionary of attribute of the given name"""
        #print('Getting attribute %s.' % name)
        name = name.split('::')

        if stream_map:
            nodemap = self.camera.GetTLStreamNodeMap()
        else:
            nodemap = self.camera.GetNodeMap()
        node = nodemap.GetNode(name[-1])

        if PySpin.IsAvailable(node) and PySpin.IsReadable(node):
            if node.GetPrincipalInterfaceType() == PySpin.intfIInteger:
                return PySpin.CIntegerPtr(node).GetValue()
            elif node.GetPrincipalInterfaceType() == PySpin.intfIFloat:
                return PySpin.CFloatPtr(node).GetValue()
            elif node.GetPrincipalInterfaceType() == PySpin.intfIBoolean:
                return PySpin.CBooleanPtr(node).GetValue()
            else:
                return PySpin.CValuePtr(node).ToString()

    def set_attributes(self, attr_dict):
        for k, v in attr_dict.items():
            self.set_attribute(k, v)

    def set_stream_attribute(self, name, value):
        self.set_attribute(name, value, stream_map=True)

    def set_attribute(self, name, value, stream_map=False):
        #print('Setting attribute %s.' % name)
        name = name.split('::')

        if stream_map:
            nodemap = self.camera.GetTLStreamNodeMap()
        else:
            nodemap = self.camera.GetNodeMap()
        node = nodemap.GetNode(name[-1])

        if PySpin.IsAvailable(node) and PySpin.IsWritable(node):
            if node.GetPrincipalInterfaceType() == PySpin.intfIInteger:
                 PySpin.CIntegerPtr(node).SetValue(value)
            elif node.GetPrincipalInterfaceType() == PySpin.intfIFloat:
                 PySpin.CFloatPtr(node).SetValue(value)
            elif node.GetPrincipalInterfaceType() == PySpin.intfIBoolean:
                PySpin.CBooleanPtr(node).SetValue(value)
            else:
                PySpin.CValuePtr(node).FromString(value)

            sleep(0.05)
            # Sometimes this doesn't work, so let's check and print warnings if it
            # fails:
            name = '::'.join(name)
            return_value = self.get_attribute(name, stream_map=stream_map)
            if return_value != value:
                print('WARNING: setting attribute %s to %s failed. '%(name, str(value)) +
                      'Returned value %s instead'%str(return_value))
            else:
                print('Successfully set %s to %s.'%(name, str(return_value)))
        else:
            print('WARNING: not capable of writing attribute %s.'%'::'.join(name))


    def snap(self):
        """Acquire a single image and return it"""
        self.configure_acquisition(continuous=False, bufferCount=1)
        #self.trigger()
        image = self.grab()
        self.camera.EndAcquisition()
        return image

    def grab(self, timeout=None):
        """Grab and return single image during pre-configured acquisition."""
        #print('Grabbing...')
        if timeout is None:
            image_result = self.camera.GetNextImage(self.timeout)
        else:
            image_result = self.camera.GetNextImage(timeout)
        img = self._decode_image_data(image_result.GetData())
        image_result.Release()
        return img

    def grab_multiple(self, n_images, images, timeouts=None):
        """Grab n_images into images array during buffered acquistion."""
        if timeouts is not None:
            if len(timeouts) != n_images:
                raise ValueError(
                    'Timeouts must be the have length n_images (%d).'%n_images+
                    'Instead, it has length %d'%len(timeouts)
                )

        for i in range(n_images):
            # Calculate the timeout for this image:
            if timeouts is None:
                timeout = self.timeout
            else:
                timeout = int(timeouts[i]*1000) + self.timeout

            # Print the initial message, if applicable:
            if i==0:
                print(f"Attempting to grab {n_images} images. Timeout {timeout/1000} s.")
            else:
                print(f"Got image {i} of {n_images}. Next timeout {timeout/1000} s.")

            # This abort acqusition effectively does nothing, becuase the 
            # the hold is at the timeout self.grab phase.
            if self._abort_acquisition:
                print("Abort during acquisition.")
                self._abort_acquisition = False
                return

            images.append(self.grab(timeout=timeout))

        print(f"Got {len(images)} of {n_images} images.")


    def trigger(self):
        """Execute software trigger"""
        nodemap = self.camera.GetNodeMap()
        trigger_cmd = PySpin.CCommandPtr(nodemap.GetNode('TriggerSoftware'))
        if not PySpin.IsAvailable(trigger_cmd) or not PySpin.IsWritable(trigger_cmd):
            print('WARNING: Unable to execute trigger. Aborting...')
        else:
            trigger_cmd.Execute()


    def configure_acquisition(self, continuous=True, bufferCount=100):
        self.pix_fmt = self.get_attribute('PixelFormat')
        self.height = self.get_attribute('Height')
        self.width = self.get_attribute('Width')

        # Unless the camera settings are set properly, in cntinuous mode
        # the camera will generally move faster than BLACS, and so the buffer
        # will fill up.  With a Flea3, I was unable to solve the prolem
        # easily.  It really is quite annoying.
        self.set_stream_attribute('StreamBufferCountMode', 'Manual')
        self.set_stream_attribute('StreamBufferCountManual', bufferCount)

        if continuous:
            self.set_stream_attribute('StreamBufferHandlingMode', 'NewestFirst')
            self.set_attribute('AcquisitionMode', 'Continuous')
        elif bufferCount == 1:
            self.set_stream_attribute('StreamBufferHandlingMode', 'OldestFirst')
            self.set_attribute('AcquisitionMode', 'SingleFrame')
        else:
            self.set_stream_attribute('StreamBufferHandlingMode', 'OldestFirst')
            self.set_attribute('AcquisitionMode', 'MultiFrame')
            self.set_attribute('AcquisitionFrameCount', bufferCount)

        self.camera.BeginAcquisition()

    def _decode_image_data(self, img):
        """Spinnaker image buffers require significant formatting.
        This returns what one would expect from a camera.
        configure_acquisition must be called first to set image format parameters."""
        if self.pix_fmt.startswith('Mono'):
            if self.pix_fmt.endswith('8'):
                dtype = 'uint8'
            else:
                dtype = 'uint16'
            image = np.frombuffer(img, dtype=dtype).reshape(self.height, self.width)
        else:
            msg = """Only MONO image types currently supported.
            To add other image types, add conversion logic from returned
            uint8 data to desired format in _decode_image_data() method."""
            raise ValueError(dedent(msg))
        return image.copy()

    def stop_acquisition(self):
        print('Stopping acquisition (camera.stop_acqusition)...')
        self.camera.EndAcquisition()

        # This is supposed to provide debugging info, but as with most things
        # in PySpin, it appears to be completely useless:.
        num_frames=self.get_attribute('StreamTotalBufferCount', stream_map=True)
        failed_frames=self.get_attribute('StreamFailedBufferCount', stream_map=True)
        underrun_frames=self.get_attribute('StreamBufferUnderrunCount', stream_map=True)
        print('Stream info: %s frames acquired, %s failed, %s underrun' %
              (str(num_frames), str(failed_frames), str(underrun_frames)))

    def abort_acquisition(self):
        print('Stopping acquisition (camera.abort_acquisition)...')
        self._abort_acquisition = True

    def close(self):
        print('Closing down the camera...')
        self.camera.DeInit()
        self.camList.Clear()
        self.system.ReleaseInstance()


class SpinnakerCameraWorker(IMAQdxCameraWorker):
    """Spinnaker API Camera Worker.

    Inherits from IMAQdxCameraWorker."""
    interface_class = Spinnaker_Camera

    def transition_to_buffered(self, device_name, h5_filepath, initial_values, fresh):
        # This method is basically the same as the IMAQdx version, except that
        # it calculates timeouts between frames to pass along to PySpin
        if getattr(self, 'is_remote', False):
            h5_filepath = path_to_local(h5_filepath)
        if self.continuous_thread is not None:
            # Pause continuous acquistion during transition_to_buffered:
            self.stop_continuous(pause=True)
        with h5py.File(h5_filepath, 'r') as f:
            # Get the camera_attributes from the device_properties
            properties = labscript_utils.properties.get(
                f, self.device_name, 'device_properties'
            )
            camera_attributes = properties['camera_attributes']
    
            self.exception_on_failed_shot = properties['exception_on_failed_shot']
            saved_attr_level = properties['saved_attribute_visibility_level']

            # Get the exposures:
            group = f['devices'][self.device_name]
            if not 'EXPOSURES' in group:
                return {}
            self.h5_filepath = h5_filepath
            self.exposures = group['EXPOSURES'][:]
            self.n_images = len(self.exposures)

            # Calculate the timeouts between images.  The first timeout is set
            # to 30 s, because it may take that long for all devices to get
            # set into buffered mode.
            self.timeouts = [properties['stop_acquisition_timeout']]
            for i in range(1, len(self.exposures)):
                self.timeouts.append(self.exposures['t'][i] - self.exposures['t'][i-1])

            self.stop_acquisition_timeout = np.sum(self.timeouts)

        # Only reprogram attributes that differ from those last programmed in, or all of
        # them if a fresh reprogramming was requested:
        if fresh:
            self.smart_cache = {}
        self.set_attributes_smart(camera_attributes)
        # Get the camera attributes, so that we can save them to the H5 file:
        if saved_attr_level is not None:
            self.attributes_to_save = self.get_attributes_as_dict(saved_attr_level)
        else:
            self.attributes_to_save = None
        print(f"Configuring camera for {self.n_images} images.")
        self.camera.configure_acquisition(continuous=False, bufferCount=self.n_images)
        self.images = []
        self.acquisition_thread = threading.Thread(
            target=self.camera.grab_multiple,
            args=(self.n_images, self.images, self.timeouts),
            daemon=True,
        )
        self.acquisition_thread.start()
        return {}

    def abort(self):
        if self.acquisition_thread is not None:
            self.camera.abort_acquisition()
            self.acquisition_thread.join()
            self.acquisition_thread = None
            self.camera.stop_acquisition()
        self.camera._abort_acquisition = False
        self.images = None
        self.n_images = None
        self.attributes_to_save = None
        self.exposures = None
        self.acquisition_thread = None
        self.h5_filepath = None
        self.stop_acquisition_timeout = None
        self.exception_on_failed_shot = None
        # Resume continuous acquisition, if any:
        if self.continuous_dt is not None and self.continuous_thread is None:
            self.start_continuous(self.continuous_dt)
        return True

    def abort_buffered(self):
        print('Abort buffered called.')
        return self.abort()
    
    def abort_transition_to_buffered(self):
        print('Abort transition to buffered called.')
        return self.abort()

    #def continuous_loop(self, dt):
    #    """Acquire continuously in a loop, with minimum repetition interval dt"""
    #    self.camera.trigger()
    #    while True:
    #        if dt is not None:
    #            t = perf_counter()
    #        image = self.camera.grab()
    #        self.camera.trigger()
    #        self._send_image_to_parent(image)
    #        if dt is None:
    #            timeout = 0
    #        else:
    #            timeout = t + dt - perf_counter()
    #        if self.continuous_stop.wait(timeout):
    #            self.continuous_stop.clear()
    #            break