# Linux-DVB-S2 Example

This is just a quick example of using the Linux DVB API to access a connected sat-tuner. 

It is very basic but shows how to open, tune and get status from your tuner device. 

It also shows how to setup a DEMUX filter to finally read filtered data from the tuner device. 

Probably a good starting point to get acquainted with the basic Linux Kernel DVB API Framework.(without the LibDVBV5 interface) 

There is only one Sourcefile and you can build easy like this: 
 
cc DVBdemo.c -o dvb   (or whatever name suits you) 

Then run it:  ./dvb 

There are no parameters as everything is hardcoded for simplicity. 

It assumes you have your Sat-Tuner connected via LNB to the Astra satelite. 

If everything runs well, RTL Teletext data is saved in a file named "ttdata.bin" .
