/*

basic Linux kernel DVB V5.5 API , no libdvbv5!

This code opens adapter0 DVB, expecting a SATreceiver device connected to linux.
I am using Astra 19.2 in Germany.


2.5. Frontend Function Calls

    2.5.1. DVB frontend open()
    2.5.2. DVB frontend close()
    2.5.3. ioctl FE_GET_INFO
    2.5.4. ioctl FE_READ_STATUS
    2.5.5. ioctl FE_SET_PROPERTY, FE_GET_PROPERTY
    2.5.6. ioctl FE_DISEQC_RESET_OVERLOAD
    2.5.7. ioctl FE_DISEQC_SEND_MASTER_CMD
    2.5.8. ioctl FE_DISEQC_RECV_SLAVE_REPLY
    2.5.9. ioctl FE_DISEQC_SEND_BURST
    2.5.10. ioctl FE_SET_TONE
    2.5.11. ioctl FE_SET_VOLTAGE
    2.5.12. ioctl FE_ENABLE_HIGH_LNB_VOLTAGE
    2.5.13. ioctl FE_SET_FRONTEND_TUNE_MODE

--
Devices in /dev/dvb/:
DVB_DEVICE_FRONTEND = Digital TV frontend. Input of tuning settings, output of status and Transport Stream TS.
DVB_DEVICE_DEMUX = Digital TV demux. Demux Transport Stream TS into elementary streams by Pid. Also output.
DVB_DEVICE_DVR 0 Digital TV Digital Video Record (DVR). Output of Demux to be used as Playback/Recording source, or Input.
DVB_DEVICE_CA = Digital TV Conditional Access (CA).
DVB_DEVICE_NET = Digital TV network. IP-stream in MPEG TS. used for IP over MPEG/Sat connectivity. protocols MPE ULE.
                  you need to specify the Pid in the MPEG TS, where the IP data is transmitted.
				  cannot be used to stream TV from your frontend adapter.!
Depreciated in version 5:
DVB_DEVICE_VIDEO = Digital TV video decoder. Deprecated. Used only on av7110-av.
DVB_DEVICE_AUDIO = Digital TV audio decoder. Deprecated. Used only on av7110-av.
DVB_DEVICE_OSD = Digital TV On Screen Display (OSD). Deprecated. Used only on av7110--
DVB_DEVICE_SEC = Digital TV standalone Common Interface (CI). Deprecated. now in frontend.

Possible  stream  types  are :
TS = transport  streams
PS = program streams
PES = packetized elementary streams
SECTIONS = service information streams
PMT = Program Map Table, verweist auf pids im TS

other shortcuts:
SEC = Satelite equipment control = Diseq = LNB control
INVERSION = IQ signal from tuner inverted
TS = MPEG Transport Stream, containing multible elementary substreams.

Demultiplexing:
The Digital TV demux device filters the MPEG-TS stream for the digital TV. If the Tuner-driver and hardware supports it, those filters are implemented in the Tuner-hardware. Otherwise, the Kernel provides a software emulation.
So it can be a hardware demuxer if present in the TV adaptercard, OR it uses the software demuxer in the kernel.
Most modern Tuner-hardware do not support Hardware demuxing any more, but send the complete MPEG Transport Stream TS from the Transponder.
This software demuxer sourcecode in the kernel is in DMXDEV.C !!
In my case, its the software demuxer being used, so the kernel does the demuxing. how nice.

The DEMUX Device (kernel-software) fishes out elementary substreams from the TS that can be red by reading the DEMUX filter device,
or the DVR device, depending on how the demux-filter was setup.
Multible filters can be set up each having its own file-handle.
Fe. to just read the teletext stream, one might setup a filter with teletext-Pid, output type DMX_OUT_TAP, 
    to be red from the DEMUX-Dev-filter file handle.
Fe. if video and audio Pids have been set as filter in the DEMUX device, and output type is DMX_OUT_TS_TAP, all can be red from DVR-Dev.
	This will assemble video and audio into a new TS just containing these two streams.
	This TS can be fed into VLC to be watched on the computer.(read from DVR device)
	Or to ffmpeg for transcoding further.
So there is no magic behind the DEMUX device ;)	

my old sunxi kernel 3.4 has dvb api version 5.4 (This on a BananaPi M1)

Have fun, xenpac ;)

Jan 2022, Thomas Krueger, Germany
*/


#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <linux/dvb/dmx.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>

// setup adapter devices to use
#define DEMUX "/dev/dvb/adapter0/demux0"
#define FRONTEND "/dev/dvb/adapter0/frontend0"

// global data:

int Pid=105; //teletext pid of station				   
// TV channel to tune to:
//char Name[]="RTL";
uint32_t Frequency  = 12188000; // Khz.  Transponder Frequency!
uint32_t Symbolrate = 27500000; // in Baud
int Pol_vert = 0; // 1=vert, 0=Hor, voltage 13/18
fe_code_rate_t Fec = FEC_3_4; // maybe FEC_auto works also.
fe_rolloff_t  Rolloff = ROLLOFF_35; // not really necessary as its default for dvb-s
fe_modulation_t Modulation = QPSK; // PSK_8 for HD
fe_delivery_system_t Delsys = SYS_DVBS; // SYS_DVBS or SYS_DVBS2
// see frontend.h for the enum definitions.

// LNB Stuff:>>
// Astra range: 10,70 und 12,75 GHz.  (Low-Band) 10,70–11,70 GHz;  (High-Band) 11,70–12,75 GHz.
// Downlink frequency range on the coax-cable: 950 - 2150 Mhz = Tuner input frequency range
// all values in KHZ!!!
uint32_t Loflow    = 9750000;  // lnb local oscillator frequency for lowband, downmixer, controled by 22Khz tone
uint32_t Lofhigh   = 10600000; // lnb local oscillator frequency for highband, downmixer, controled by 22Khz tone
uint32_t Lnblow    = 10700000; // lnb min frequency of reception ( actually Astra range, LNB might do more)
uint32_t Lnbhigh   = 12750000; // lnb max frequency of reception
uint32_t Lnbswitch = 11700000; // the frequency to switch high/low band. if freq >= Lnbswitch, then switch to highband, else lowband.


int frontfd; // file handle frontend device
int demuxfd; // file handle of demux device
struct dvb_frontend_info info;

uint8_t buf[1840]; // receive buffer. multible of 184=one ttpage, thats for teletext.

//protos:
void getcaps(void);
int getstatus(void);
int tune(void);
int create_dmxfilter(int pid, int pes_type);

void main(void)
{
    int bytes;
    int file;
	int i;

    // open the frontend
    frontfd = open(FRONTEND, O_RDWR|O_NONBLOCK);
    if (frontfd < 0)
    {
        printf("Device busy or not connected\n");
        return;
    }

    // get capabilites of the adapter
    getcaps();
    if (!tune()) return;
    if (!getstatus()) return;

    //while (1) sleep(100); //stay opened, if you want to use another program to read from DVR device, thats like szap-s2

    // create a filter for the teletext stream
    demuxfd=create_dmxfilter(Pid, DMX_PES_TELETEXT); //DMX_PES_TELETEXT, DMX_PES_OTHER
    if (!demuxfd)
    {
        printf("set filter failed\n");
        return;
    }

    file = open("ttdata.bin",O_TRUNC|O_WRONLY|O_CREAT,0666);
    if (file == -1)
    {
        perror("cannot open file:");
        return;
    }

i=10;  // read 10 buffers full of data into file, then exit
    while(i--)
    {
        bytes = read(demuxfd, buf, 1840);// sizeof(buf)); // read multible of 184 bytes= transmitted teletext chunks
        if (bytes < 0)
        {
            perror("Read error:");
        }
        else
        {
            printf("got: %d Bytes\n",bytes);
            write(file,buf,bytes);
        }
    }

// usually file handles are closed automaticly when process exits.
    close(frontfd);
    close(demuxfd);
    close(file);

// BYE

}

void getcaps(void)
{
    printf("\n----------- DVB adapter info --------------\n");

    printf("found Linux Kernel DVB API Version %d.%d\n",DVB_API_VERSION,DVB_API_VERSION_MINOR); //from include/linux/dvb/version.h

    if (ioctl(frontfd, FE_GET_INFO, &info))
    {
        printf("getinfo failed\n");
        return;
    }
    printf("found adapter: %s\n",info.name);

    switch (info.type)
    {
    case FE_QPSK:
        printf("Type: QPSK DVB-S\n");
        break;
    case FE_QAM:
        printf("Type: QAM\n");
        break;
    case FE_OFDM:
        printf("Type: OFM\n");
        break;
    case FE_ATSC:
        printf("Type: ATSC\n");
        break;
    default:
        printf("Type: unknown %d\n",info.type);
        break;
    }

    printf("Tuner min Freqeuncy: %lu Khz\n",info.frequency_min); // Note: the tuner frequency is NOT the Sat frequency!! downmixed.
    printf("Tuner max Freqeuncy: %lu Khz\n",info.frequency_max);
    printf("tuning stepsize: %lu Khz\n",info.frequency_stepsize); // One might try a full scan with this stepsize ?!
    printf("tuning Frequency tolerance: %lu Khz\n",info.frequency_tolerance); // thats the capture range of the AFC. 5Mhz in my case.
    // AFC: typical 5Mhz max can be the differnce between nominal and actual transponder frequency to still tune correct. LNB inacuracy!
    printf("min Symbolrate: %lu Sym/s\n",info.symbol_rate_min);
    printf("max Symbolrate: %lu Sym/s\n",info.symbol_rate_max);
    printf("Symbolrate tolerance ppm: %lu\n",info.symbol_rate_tolerance);

    //capabilites:
    if (info.caps&FE_CAN_INVERSION_AUTO) printf("Inversion auto\n");
    if (info.caps&FE_CAN_FEC_1_2) printf("FEC1/2\n");
    if (info.caps&FE_CAN_FEC_2_3) printf("FEC2/3\n");
    if (info.caps&FE_CAN_FEC_3_4) printf("FEC3/4\n");
    if (info.caps&FE_CAN_FEC_4_5) printf("FEC4/5\n");
    if (info.caps&FE_CAN_FEC_5_6) printf("FEC5/6\n");
    if (info.caps&FE_CAN_FEC_6_7) printf("FEC6/7\n");
    if (info.caps&FE_CAN_FEC_7_8) printf("FEC7/8\n");
    if (info.caps&FE_CAN_FEC_8_9) printf("FEC8/9\n");
    if (info.caps&FE_CAN_FEC_AUTO) printf("FEC auto\n");
    if (info.caps&FE_CAN_QPSK) printf("QPSK DVB-S\n");
    if (info.caps&FE_CAN_QAM_16) printf("QAM16\n");
    if (info.caps&FE_CAN_QAM_32) printf("QAM32\n");
    if (info.caps&FE_CAN_QAM_64) printf("QAM64\n");
    if (info.caps&FE_CAN_QAM_128) printf("QAM128\n");
    if (info.caps&FE_CAN_QAM_256) printf("QAM256\n");
    if (info.caps&FE_CAN_QAM_AUTO) printf("QAM auto\n");
    if (info.caps&FE_CAN_TRANSMISSION_MODE_AUTO) printf("Transmission mode auto\n");
    if (info.caps&FE_CAN_BANDWIDTH_AUTO) printf("Bandwidth auto\n");
    if (info.caps&FE_CAN_GUARD_INTERVAL_AUTO) printf("Quard Interval auto\n");
    if (info.caps&FE_CAN_HIERARCHY_AUTO) printf("Hierarchy auto\n");
    if (info.caps&FE_CAN_8VSB) printf("8VSB\n");
    if (info.caps&FE_CAN_16VSB) printf("16VSB\n");
    if (info.caps&FE_HAS_EXTENDED_CAPS) printf("Has extended Caps!\n");
    if (info.caps&FE_CAN_TURBO_FEC) printf("Turbo FEC\n");
    if (info.caps&FE_CAN_2G_MODULATION) printf("2G-Modulation = DVB-S2\n");
    if (info.caps&FE_CAN_RECOVER) printf("can recover from cable unplug\n");
    if (info.caps&FE_CAN_MUTE_TS) printf("can stop spurious TS data output\n");
    printf("----------- EndOf DVB adapter Info --------------\n\n\n");
}

// get tune status. exit: 0=fail,not tuned; 1=OK, tuned
int getstatus(void)
{
    fe_status_t status;
    uint16_t snr, signal;
    uint32_t ber, uncorrected_blocks;
    int timeout = 0;

    struct dtv_property p[] =   // setting up 6 commands in the array
    {
        { .cmd = DTV_DELIVERY_SYSTEM },
        { .cmd = DTV_MODULATION },
        { .cmd = DTV_INNER_FEC },
        { .cmd = DTV_ROLLOFF },
        { .cmd = DTV_FREQUENCY },
        { .cmd = DTV_SYMBOL_RATE },
    };
    struct dtv_properties cmdseq =
    {
        .num = 6,  // array of 6
        .props = p // pointer to the array
    };

    do
    {
        if (ioctl(frontfd, FE_READ_STATUS, &status) == -1)
            perror("FE_READ_STATUS failed");
        /* some frontends might not support all these ioctls, thus we
         * avoid printing errors
         */
        if (ioctl(frontfd, FE_READ_SIGNAL_STRENGTH, &signal) == -1)
            signal = -2;
        if (ioctl(frontfd, FE_READ_SNR, &snr) == -1)
            snr = -2;
        if (ioctl(frontfd, FE_READ_BER, &ber) == -1)
            ber = -2;
        if (ioctl(frontfd, FE_READ_UNCORRECTED_BLOCKS, &uncorrected_blocks) == -1)
            uncorrected_blocks = -2;


        printf ("status %02x | signal %3u%% | snr %3u%% %d | ber %d | unc %d | ",
                status, (signal * 100) / 0xffff, (snr * 100) / 0xffff, snr, ber, uncorrected_blocks);


        if (status & FE_HAS_LOCK)
            printf("FE_HAS_LOCK");
        printf("\n");

        if ((status & FE_HAS_LOCK) || (++timeout >= 10))
            break;

        usleep(1000000);
    }
    while (1);  // wait 5 iterations for tuned


    if (status & FE_HAS_LOCK)
    {
        printf("....get_status done, tune OK\n");
	// print the finally tuned frequency in the tuner:
		if ((ioctl(frontfd, FE_GET_PROPERTY, &cmdseq)) == -1) // get final tuning parameters
		{
			perror("FE_GET_PROPERTY failed");
		}
		//	printf("delivery 0x%x, ", p[0].u.data);
		//	printf("modulation 0x%x\n", p[1].u.data);
		//	printf("coderate 0x%x, ", p[2].u.data);
		//	printf("rolloff 0x%x\n", p[3].u.data);
			printf("final Tuned Frequency %d Khz\n", p[4].u.data);
  	
        return 1;
    }
    else
        printf("....get_status done, tune failed\n");


    return 0;
}


// return: 0=fail; 1=OK
int tune(void)
{
    struct dvb_frontend_event ev;
    fe_sec_tone_mode_t tone;
    fe_sec_voltage_t volt;
    uint32_t tunerfreq;
    // first set LNB voltage

    if (Frequency >= Lnbswitch) // if Highband
    {
        tone=SEC_TONE_ON;
        tunerfreq = Frequency - Lofhigh;
    }
    else //Lowband
    {
        tone = SEC_TONE_OFF;
        tunerfreq = Frequency - Loflow;
    }

    if (ioctl(frontfd, FE_SET_TONE,  tone) == -1)
        printf("Set 22khz tone failed\n");

    if (Pol_vert)
        volt = SEC_VOLTAGE_13;
    else
        volt = SEC_VOLTAGE_18;

    if (ioctl(frontfd,  FE_SET_VOLTAGE, volt) == -1)
        printf("Set voltage failed\n");


    // define a set of 9 frontend commands as array of struct dtv_property
    // we use global value defined on top for the wanted channel
    struct dtv_property p[] =
    {
        { .cmd = DTV_DELIVERY_SYSTEM,	.u.data = Delsys },
        { .cmd = DTV_FREQUENCY,		.u.data = tunerfreq },
        { .cmd = DTV_MODULATION,	.u.data = Modulation },
        { .cmd = DTV_SYMBOL_RATE,	.u.data = Symbolrate },
        { .cmd = DTV_INNER_FEC,		.u.data = Fec },
        { .cmd = DTV_INVERSION,		.u.data = INVERSION_AUTO },
        { .cmd = DTV_ROLLOFF,		.u.data = Rolloff },
        { .cmd = DTV_PILOT,		.u.data = PILOT_AUTO },
        { .cmd = DTV_TUNE },
    };
    struct dtv_properties cmdseq =
    {
        .num = 9,  //array size of the pointer below
        .props = p  // pointer to an array of struct dtv_property
    };

    // discard stale QPSK events. needs nonblock flag on frondend open! and maybe DTV_CLEAR command 


    while (1)
    {
        if (ioctl(frontfd, FE_GET_EVENT, &ev) == -1)
            break;
    }

    // finally execute the tune command
    printf("tuning to: %d Mhz, errors Ber and Unc should be 0,HasLock...\n",tunerfreq/1000);
    if ((ioctl(frontfd, FE_SET_PROPERTY, &cmdseq)) == -1)
    {
        perror("Set tune command failed");
        return 0;
    }

    return 1;
}



/*
Create a filehandle to a demux-filter.
The filter input is the TS stream from the adaptercard. ie. the full Transportstream TS. DMX_IN_FRONTEND
If input is DMX_IN_DVR, then TS transportstream is inputted into the DVR device (fe.playback from file)
The output can be red from the filehandle return here. ie. the filtered data can be red without being reassembled into another TS stream.
It uses the DMX_OUT_TAP filter option.
If DMX_OUT_TS_TAP is used, the output of the filter is assembled into a new TS stream (together with all other active filters) being outputted on the DVR device.

A DEMUX filter extracts a stream with PID from the inputted MPEG TS stream. (DMXDEV.C)

- pid = the wanted pid value to filter out
- pes-type = type of that pid, if that matters
ret: 0=fail,else the filehandle created
*/
int create_dmxfilter(int pid, int pes_type)
{
    struct dmx_pes_filter_params pesfilter;
    int fd;

// set the output buffer of the demux device to a specific size. not used
    /*
	//This replaces the existing stream output ringbuffer with the below new one for that filter.
	// The default ringbuffersize is (10*188*1024)=1,9MB

    		int buffersize = 64 * 1024;
    		fprintf(stderr, "Setting DMX_SET_BUFFER_SIZE %d\n", buffersize);
    		if (ioctl(dmxfd, DMX_SET_BUFFER_SIZE, buffersize) == -1)
    			perror("DMX_SET_BUFFER_SIZE failed");
    */

    //open a new demux device filter instance
    if ((fd = open(DEMUX, O_RDWR)) < 0)
    {
        printf("opening demux device failed\n");
        return 0;
    }


    pesfilter.pid = pid;  // the pid we want to be filtered out of the TS stream
    pesfilter.input = DMX_IN_FRONTEND; // filter input is the TS stream from the tuner
    pesfilter.output = DMX_OUT_TAP ; //DMX_OUT_TS_TAP=out multiplexed to DVR; DMX_OUT_TAP=out on the filter nnmultiplexed
    pesfilter.pes_type = pes_type;  // the type of data to be filtered out
    pesfilter.flags = DMX_IMMEDIATE_START; // start without giving the start command

    if (ioctl(fd, DMX_SET_PES_FILTER, &pesfilter) == -1)
    {
        perror("DMX_SET_PES_FILTER failed.");

        return 0;
    }

    return fd;
}