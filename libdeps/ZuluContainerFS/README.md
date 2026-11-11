# ZuluContainerFS
## Seamless image data access of container formats based on top of SdFat's FsFile class
This library is designed for the ZuluSCSI and ZuluIDE to autodetect
different container formats and present the container file's image
data seamlessly without having to deal with containers metadata.

To do this ZCFileFs inherits from SdFat's FsFile class. Although it
can't override FsFile's methods, it replaces the necessary calls with
the same method signature needed for image access to work correctly.
So calls like open, and close are the same as they would be with FsFile.

## Supported Containers
Currently only Microsoft's vhd with fixed data (full allocation of image
size) is supported

## License
The library is [licensed under MIT license](LICENSE.md).
