///////////////////////////////////////
/// sjz38, dap263
/// 640x480 version!
/// Mandelbrot mouse interaction
/// compile with
/// gcc mandelbrot_mouse.c -lm
/// Shell code from ECE 5760 website, mouse code baed on:
/// http://stackoverflow.com/questions/11451618/how-do-you-read-the-mouse-button-state-from-dev-input-mice
///////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
 
// #include <fcntl.h>
#include <pthread.h>
#include "address_map_arm_brl4.h"
 
/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_disc (int, int, int, short);
int  VGA_read_pixel(int, int) ;
int  video_in_read_pixel(int, int);
void draw_delay(void) ;
 
// the light weight buss base
void *h2p_lw_virtual_base;
 
// RAM FPGA command buffer
volatile unsigned int * sram_ptr = NULL ;
void *sram_virtual_base;
 
// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;
 
// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;
 
// /dev/mem file id
int fd;
 
// measure time
struct timeval t1, t2;
double elapsedTime;
struct timespec delay_time ;
 
// Int to fixed point representation factor
#define I2F 23
 
volatile int * cycles= NULL;
volatile int * x_pio = NULL;
volatile int * y_pio = NULL;
volatile int * zx_pio = NULL;
volatile int * zy_pio = NULL;
 
// Animation flag
int animate = 0;
 
float center_x_read;
float center_y_read;
char command_buf [64];
double center_x;
double center_y;
// Zoom data
int max_zoom;
int ZOOMSIZE = 1035;
int zoom = 0;
int zoom_prev = 0;
 
// Initialize locks
pthread_mutex_t read_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mouse_lock = PTHREAD_MUTEX_INITIALIZER;
// Mouse values
volatile int new_bytes = 0;
int left, middle, right;
signed char x, y;
 
// Reads mouse input
void* mouse_thread() {
    int fd, bytes;
    unsigned char data[3];
 
    const char *pDevice = "/dev/input/mice";
 
    fd = open(pDevice, O_RDWR);
    if (fd == -1) {
        printf("Error Opening %s\n", pDevice);
        return -1;
    }
    while(1) {
        bytes = read(fd, data, sizeof(data));
        if (bytes > 0) {
              left  = data[0] & 0x1;
              right = data[0] & 0x2;
              middle = data[0] & 0x4;
              x = data[1];
              y = -1*data[2];
              pthread_mutex_lock(&mouse_lock);
              new_bytes = 1;
              pthread_mutex_unlock(&mouse_lock);
        }
    }
 
    return 0;
}
 
// Reads keyboard input
void* read_thread(){
    while(1){
        scanf("%s", command_buf);
        pthread_mutex_lock(&read_lock);
        if(command_buf[0] == 'a'){
            printf("Got a\n\r");
           
            printf("Enter Center X:\n\r");
            scanf("%f", &center_x_read);
            center_x = center_x_read;
            printf("Enter Center Y:\n\r");
            scanf("%f", &center_y_read);
            center_y = center_y_read;
            printf("Enter Max Zoom:\n\r");
            scanf("%d", &max_zoom);
            max_zoom = (max_zoom == -1 || max_zoom > ZOOMSIZE) ? ZOOMSIZE : max_zoom;
            zoom = 0;
            animate = 1;
            sleep(1);
        }
        pthread_mutex_unlock(&read_lock);
    }
}
 
int main(void)
{
    delay_time.tv_nsec = 10 ;
    delay_time.tv_sec = 0 ;
 
    // Declare volatile pointers to I/O registers (volatile     // means that IO load and store instructions will be used   // to access these pointer locations,
    // instead of regular memory loads and stores)
   
    // === need to mmap: =======================
    // FPGA_CHAR_BASE
    // FPGA_ONCHIP_BASE      
    // HW_REGS_BASE        
 
    // === get FPGA addresses ==================
    // Open /dev/mem
    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 )    {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }
   
    // get virtual addr that maps to physical
    // for light weight bus
    h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
    if( h2p_lw_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap1() failed...\n" );
        close( fd );
        return(1);
    }
   
    // === get VGA char addr =====================
    // get virtual addr that maps to physical
    vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );  
    if( vga_char_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap2() failed...\n" );
        close( fd );
        return(1);
    }
   
    // Get the address that maps to the character
    vga_char_ptr =(unsigned int *)(vga_char_virtual_base);
 
    // === get VGA pixel addr ====================
    // get virtual addr that maps to physical
    // SDRAM
    vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE); //SDRAM_BASE    
   
    if( vga_pixel_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap3() failed...\n" );
        close( fd );
        return(1);
    }
    // Get the address that maps to the FPGA pixel buffer
    vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);
   
    // === get RAM FPGA parameter addr =========
    sram_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE); //fp  
   
    if( sram_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap3() failed...\n" );
        close( fd );
        return(1);
    }
    // Get the address that maps to the RAM buffer
    sram_ptr =(unsigned int *)(sram_virtual_base);
   
    // Initialize PIO connections
    cycles = (int*)(h2p_lw_virtual_base + 0x4b000);
    x_pio  = (int*)(h2p_lw_virtual_base + 0x4b010);
    y_pio  = (int*)(h2p_lw_virtual_base + 0x4b020);
    zx_pio = (int*)(h2p_lw_virtual_base + 0x4b030);
    zy_pio = (int*)(h2p_lw_virtual_base + 0x4b040);
 
    // Upper left corner coordinate to be sent to PIO
    int xn, yn;
   
    // Intermediate variables for x,y calculations
    int xcoord = 0;
    int ycoord = 0;
   
    // Calculated coordinates for printing to VGA
    float xprint, yprint;
 
    // Initialize values
    xn = -2;
    yn = -1;
    *x_pio = -2*pow(2,I2F);
    *y_pio = -1*pow(2,I2F);
    *zx_pio = 3.0/640*pow(2,I2F);
    *zy_pio = 1.0/240*pow(2,I2F);
    int XWIDTH = 3;
    int YWIDTH = 2;
 
    int TWO23 = pow(2, I2F);
 
 
    double xwidth [ZOOMSIZE];
    double ywidth [ZOOMSIZE];
    double factor;
    double xpan, ypan;
    xwidth[0] = (double)XWIDTH;
    ywidth[0] = (double)YWIDTH;
    // Generate width lookup table for different zoom levels
    for (i = 1; i < ZOOMSIZE; i++) {
        factor = pow(.99, i);
        xwidth[i] = XWIDTH*factor;
        ywidth[i] = YWIDTH*factor;
    }
    double xhps = -2;
    double yhps = -1;
    double xw = 3;
    double yw = 2;
    double xc = xw/2;
    double yc = yw/2;
 
    pthread_t thread_read;
    pthread_t thread_mouse;
 
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE);
 
    pthread_create(&thread_read, NULL, read_thread, NULL);
    pthread_create(&thread_mouse, NULL, mouse_thread, NULL);
 
    while(1) {
        if(!animate){ // Reading from mouse input
            pthread_mutex_lock(&read_lock);
            if (new_bytes) {
                pthread_mutex_lock(&mouse_lock);
                new_bytes = 0;
                pthread_mutex_unlock(&mouse_lock);
 
                zoom_prev = zoom;  
                if (left && zoom < ZOOMSIZE) zoom++;
                else if (right && zoom) zoom--;
                if (zoom != zoom_prev) { // zoom changed
                    xw = xwidth[zoom];
                    yw = ywidth[zoom];
                    xc = (xwidth[zoom_prev] - xw)/2;
                    yc = (ywidth[zoom_prev] - yw)/2;
                    xhps = xhps + xc/2;
                    yhps = yhps + yc/2;
                    xpan = x*xw/640;
                    ypan = y*yw/480;
                }  
                xhps   += x*xw/640;
                yhps   += y*yw/480;
                *x_pio  = xhps*TWO23;
                *y_pio  = yhps*TWO23;
                *zx_pio = (xwidth[zoom]*TWO23/640);
                *zy_pio = (ywidth[zoom]*TWO23/480);
                double cx = xhps+(0.5*xwidth[zoom]);
                double cy = yhps+(0.5*ywidth[zoom]);
                printf("zoom=%d, center=(%8.7f, %8.7f), xhps=%8.7f, yhps=%8.7f, time=%d msec \n\r",
                        zoom, cx, cy, xhps, yhps, (*cycles)/100000);
            }
            pthread_mutex_unlock(&read_lock);
        }
        else if (animate){ // Running animation
            pthread_mutex_lock(&read_lock);
            if(zoom == max_zoom-1){
                printf("Reached max zoom size\n\r");
                animate = 0;
            }
            else{
                zoom_prev = zoom;
                zoom++;
                xhps = center_x - 0.5*xwidth[zoom];
                yhps = center_y - 0.5*ywidth[zoom];
 
                *x_pio = xhps*TWO23;
                *y_pio = yhps*TWO23;
                *zx_pio = (xwidth[zoom]*TWO23/640);
                *zy_pio = (ywidth[zoom]*TWO23/480);
 
                printf("zoom=%d, center=(%8.7f, %8.7f), xhps=%8.7f, yhps=%8.7f, time=%d msec \n\r",
                        zoom, center_x, center_y, xhps, yhps, (*cycles)/100000);
                int draw_time = (*cycles)/100000;
                if (draw_time < 35) draw_time = 35;
                usleep(draw_time*100);
            }
            pthread_mutex_unlock(&read_lock);
        }
        usleep(10);
    } // end while(1)
    pthread_join(thread_read, NULL);
    return 0;
} // end main
