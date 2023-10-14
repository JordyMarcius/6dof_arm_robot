FILE *fserial;
int baud, parity;
int fd;


int open_port(void) {
	int fd;		// File Descriptor for serial port
	char s[60];
	
	printf("Checking communication port : /dev/com2\n",s);
	//fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); // khusus linux
	fd = open("/dev/com2", O_RDWR | O_NOCTTY | O_NDELAY); // khusus windows
	if(fd == -1) {
		perror("-- Unable to open /dev/com --");
		exit(1);
	}	else {
		fcntl(fd, F_SETFL, 0);
		printf("...Done\n");
	}
	
	return(fd);
}

void init_port(int fd) {
	struct termios configs;
	baud=38400;
	parity=0;
	
	// Ambil konfigurasi sekarang dari ttyS0 (COM1)
	tcgetattr(fd, &configs);
	switch(baud) {
		case 9600: baud=B9600;break;
		case 19200: baud=B19200;break;
		case 38400: baud=B38400;break;
		default: baud=B19200;
	}
	cfsetispeed(&configs, baud);
	cfsetospeed(&configs, baud);
	
	// Setting Control Mode
	switch (parity) {
		case 1:  // even parity
			 configs.c_cflag |= PARENB;		// Parity enable
			 configs.c_cflag &= ~PARODD;		// Even Parity
			 break;
		case 2:  // odd parity
			 configs.c_cflag |= PARENB;		// Parity enable
			 configs.c_cflag |= PARODD;		// Even Parity
			 break;
		default : // no parity
			 //configs.c_cflag |= PARENB;		// Parity enable
			 //configs.c_cflag &= ~PARODD;		// Even Parity
			 configs.c_cflag &= ~PARENB;  // Parity disable
	}
	configs.c_cflag &= ~CSTOPB;		// 1 stop bit
	//configs.c_cflag |= CSTOPB;		// 2 stop bit
	configs.c_cflag &= ~CSIZE;
	configs.c_cflag |= CS8;			// 8-bit data
	configs.c_cflag &= ~CRTSCTS;	// Disable hardware flow control
	
	// Setting Local Mode
	configs.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	//RAW input
	
	// Setting Input mode
	configs.c_iflag |= INPCK;			// Enabling Parity Check
	configs.c_iflag &= ~ISTRIP;			// Disable strip off 8th bit when received
	configs.c_iflag &= ~(IXON | IXOFF | IXANY);	// Disable software flow control
	
	// Setting Output Mode
	configs.c_oflag &= ~OPOST;		// RAW output
	
	// Setting Control chars
	configs.c_cc[VTIME] = 0;		// Timeout in ds, 0 (no timeout)
	configs.c_cc[VMIN] = 1;
	
	// Write konfigurasi yang baru ke ttyS0
	if (tcsetattr(fd, TCSANOW, &configs)!=0) {
		fprintf(stderr, "ERROR ! Tdk bisa menulis konfigurasi pd hardware\n--> Jika hasil grafik ngaco, tutup program dan jalankan lagi\n");
	}
}

