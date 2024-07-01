#define sda PIO_PC3
#define scl PIO_PC1

#define START_TIMER() \
  do { \
    PIO->PIO_CODR = sda; \
    PIO->PIO_SODR = sda; \
  } while (0);

#define STOP_TIMER() \
  do { \
    PIO->PIO_CODR = scl; \
    PIO->PIO_SODR = scl; \
  } while (0);

void setup() {
  PMC->PMC_PCER0 |= PMC_PCER0_PID11;

	PIOC->PIO_PER = PIO_PC1;
	PIOC->PIO_PER = PIO_PC3;

	PIOC->PIO_OER = PIO_PC1;
	PIOC->PIO_OER = PIO_PC3;

	PIOC->PIO_PUDR = PIO_PC1;
	PIOC->PIO_PUDR = PIO_PC3;
}

void loop() {
  START_TIMER();
  // hmac
  STOP_TIMER();

  START_TIMER();
  // poly1305
  STOP_TIMER();
}
