GDB OpenCV imshow

This is a simple python extension script to visualize interactively OpenCV
images while debugging with gdb.

INSTALL

 You need to have gdb (version 7.2 or newer).

 You can try to use the cv_imshow command by sourcing it first from within a gdb
 session with the command

 > source cv_imshow.py

 (insert the path to the cv_imshow.py file on your machine, if it is not in the
 gdb current working directory). You can also add the command to your .gbdinit
 file if you do not want to source it at each session.

USAGE

 Usage is extremely simple, once you have sourced the file. If the variable you
 want to inspect (i.e., show) is a cv::Mat or an IplImage with name 'image', all
 you need to do is

 > cv_imshow image

 from within your gdb session.
