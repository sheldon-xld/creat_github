#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
//#include <asm/system.h>
#include <asm/uaccess.h>
//#include <asm/signal.h>
#include <asm/gpio.h>
 
#include <asm/irq.h>
#include <linux/time.h>
#include <linux/irq.h> 
#include <linux/gpio.h>
//#include <plat/dma.h>
//#include <plat/board.h>
//#include <linux/common.h>
#include <linux/of_platform.h>
#include <linux/omap-gpmc.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/atomic.h>


#define GPMC_CS0_OFFSET		0x60
#define GPMC_CS_SIZE		0x30
#define GPMC_CS_CONFIG1		0x00
#define GPMC_CS_CONFIG2		0x04
#define GPMC_CS_CONFIG3		0x08
#define GPMC_CS_CONFIG4		0x0c
#define GPMC_CS_CONFIG5		0x10
#define GPMC_CS_CONFIG6		0x14
#define GPMC_CS_CONFIG7		0x18
#define GPMC_CONFIG7_CSVALID		(1 << 6)
#define GPMC_BASE_ADDR          	0x50000000

/* GPMC register offsets */
#define GPIO_TO_PIN(bank,gpio) (32*(bank)+(gpio))
#define dsp1_rst_gpio	 			GPIO_TO_PIN(2,12) //T3
#define dsp2_rst_gpio				GPIO_TO_PIN(2,13) //T4
#define	dsp_hcntl1  				GPIO_TO_PIN(2,7) //R2
#define	dsp_hcntl0  				GPIO_TO_PIN(2,8) //R3
#define	dsp_has    				GPIO_TO_PIN(2,11) //T2
#define	dsp_hhwil  				GPIO_TO_PIN(2,10) //T1
#define fpga_reset                   		GPIO_TO_PIN(3,20) //D13
#define DEVICE_NAME "load_program"
#define BUF_LENGTH 1024
#define GPMC_CS 2

#define  sca_hpi_major 242
#define  sca_hpi_minor 0
static void __iomem *hpi_base;
static void __iomem *gpmc_base ;
unsigned long mem_base;

struct sca_hpi_driver{
	volatile unsigned short data; 
	volatile unsigned short flag;
};
struct sca_hpi_driver *tongdao[3];

struct sca_ld_config{
	volatile unsigned short data_L;
	volatile unsigned short data_H;
	volatile unsigned short ld;
	volatile unsigned short length;
	volatile unsigned short status;
	volatile unsigned short stop_flag;
	volatile unsigned short rapidio_start;
};
struct sca_ld_config *ld_tongdao;

struct sca_hpi_dev {  
	struct semaphore sem;
	unsigned char *hpi_buff;
	unsigned short *ld_buff;
	unsigned int *read_buff;
	unsigned int write_count_dsp1;
	unsigned int write_count_dsp2;
};
struct sca_hpi_dev dsp[2];
struct sca_hpi_dev fpga;
struct sca_hpi_dev ld_config;


static struct class *sca_hpi_class;
static struct cdev sca_hpi_Cdev;	/* Char device structure */

void HPI_DATAwrite(unsigned short data)
{
	volatile unsigned short *base = (unsigned short*)hpi_base;
	*base=data;
}
unsigned short HPI_DATAread(void)
{
	volatile unsigned short *base = (unsigned short*)hpi_base; 
	unsigned short temp;
	temp=*base;
	return temp;
}

unsigned int readHPIA(void)
{
	unsigned int addrH,addrL;
	unsigned int addr;

	gpio_direction_output(dsp_hcntl0,1);  //HCNTL0 low 
	gpio_direction_output(dsp_hcntl1,0);  //HCNTL1 high
	udelay(50);
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low 
	udelay(50);
	addrH=HPI_DATAread();
	gpio_direction_output(dsp_hhwil,1);  //Hhwil low 
	udelay(50);
	addrL=HPI_DATAread();

	addr = (unsigned int)addrH<<16 | (unsigned int)addrL;
	return addr;
}	

void  write6416HPIA(unsigned int addr) 
{
	unsigned int addrH,addrL;
	addrH=addr>>16;
	addrL=addr& 0xffff;	
	gpio_direction_output(dsp_hcntl0,1);  //HCNTL0 low 
	gpio_direction_output(dsp_hcntl1,0);  //HCNTL1 high
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low 
	udelay(5);
	HPI_DATAwrite(addrH);
	udelay(5);
	gpio_direction_output(dsp_hhwil,1);  //Hhwil high;
	udelay(5);
	HPI_DATAwrite(addrL);
}

void wirte6416hpic(unsigned int val)
{ 
	unsigned int valH,valL;
	valH=val>>16;
	valL=val& 0xffff;
	
	gpio_direction_output(dsp_hcntl1,0);  //HCNTL1 low 
	gpio_direction_output(dsp_hcntl0,0);  //HCNTL0 low
	udelay(5); 
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low 
	udelay(5);
	HPI_DATAwrite(valH);
	gpio_direction_output(dsp_hhwil,1);  //Hhwil high
	udelay(5);
	HPI_DATAwrite(valL);
} 


unsigned int read6416hpic(void)
{
	unsigned int hpidataH,hpidataL;
	unsigned int hpiData;
//	printk("in read6416hpic\n");
	gpio_direction_output(dsp_hcntl0,0);  //HCNTL0 low 
	gpio_direction_output(dsp_hcntl1,0);  //HCNTL1 low 
	udelay(5);
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low /*half word first select*/
	udelay(5);
	hpidataH=HPI_DATAread( );
//	printk("hpidataH is%x\n",hpidataH);
	gpio_direction_output(dsp_hhwil,1);  //Hhwil high /*half word second select*/
	udelay(5);
	hpidataL=HPI_DATAread();
//	printk("hpidataL is%x\n",hpidataL);
	hpiData=(unsigned int )hpidataH<<16|(unsigned int)hpidataL;
	return hpiData;
}

unsigned int read6416hpid(void)
{
	unsigned int hpidataH,hpidataL;
	unsigned int hpiData;
	
	gpio_direction_output(dsp_hcntl0,1);  //HCNTL0 high 
	gpio_direction_output(dsp_hcntl1,1);  //HCNTL1 high
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low 
	
	//  udelay(655);
	udelay(50);
	hpidataH=HPI_DATAread();
//	printk("hpidataH is %x\n",hpidataH);
	udelay(50);
	gpio_direction_output(dsp_hhwil,1);  //Hhwil high 
	hpidataL=HPI_DATAread();
//	printk("hpidataL is %x\n",hpidataL);
	hpiData=(unsigned int )hpidataH<<16|(unsigned int)hpidataL;
	return hpiData; 
}

unsigned int hpi6416FixRead(unsigned int addr)
{
	unsigned int temp;
	write6416HPIA(addr);
	udelay(500);
	temp=read6416hpid();
	return temp;
}
/*******************************************************************************************************************
*
*非自增模式
*
*******************************************************************************************************************/   
void hpi6416FixWrite(unsigned int addr,unsigned int val)
{
	unsigned int valH,valL;
	valH=val>>16;
	valL=val& 0xffff;
	write6416HPIA(addr);
	
	gpio_direction_output(dsp_hcntl0,1);  //HCNTL0 high
	gpio_direction_output(dsp_hcntl1,1);  //HCNTL1 high
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low   
	udelay(5);
	HPI_DATAwrite(valH);
//	printk("valH=%x\n",valH);
	udelay(5);
	gpio_direction_output(dsp_hhwil,1); 
	udelay(5);
//	printk("valL=%x\n",valL);
	HPI_DATAwrite(valL);
}

void hpi6416AutoWrite(unsigned char *buff,unsigned long len,unsigned addr)
{
	int i;
	unsigned short templ,temph;
	write6416HPIA(addr);
	
	gpio_direction_output(dsp_hcntl0,0);  //HCNTL0 low
	gpio_direction_output(dsp_hcntl1,1);  //HCNTL1 high
	
//	printk("len is%x\n",len);	
	/* HCNTL0 low
	HCNTL1 high
	HHWIL low */

	for(i=0;i<len;i+=4)
	{

		templ= (buff[i+1]<<8) | buff[i];
		temph= (buff[i+3]<<8)|buff[i+2];
		gpio_direction_output(dsp_hhwil,0);  //Hhwil low 
		HPI_DATAwrite((unsigned short)temph);
		udelay(5);
		gpio_direction_output(dsp_hhwil,1);  //Hhwil low  
		HPI_DATAwrite((unsigned short)templ);
	}
}
/*******************************************************************************************************************
*
*
*
*******************************************************************************************************************/ 
void hpi6416AutoIncWrite(unsigned int addr,unsigned int tab[])
{    
	int i;
	printk( "hpi6416AutoIncWrite is 0x%x and 0x%x\n",addr,tab[0]); 
	write6416HPIA(addr);
	
	gpio_direction_output(dsp_hcntl0,0);  //HCNTL0 low
	gpio_direction_output(dsp_hcntl1,1);  //HCNTL1 high
	gpio_direction_output(dsp_hhwil,0);  //Hhwil low 
	
	int len=tab[0];
	printk("len is%x\n",len);
	
	 
	/* HCNTL0 low
	 HCNTL1 high
	 HHWIL low */      
	for(i=0;i<16;i++)
	{	
		//write6416HPIA(addr+4*i);
		udelay(5);
		//HPI_DATAwrite(tab[i]>>16);
		HPI_DATAwrite((unsigned short)tab[i]);
		printk("tab[i] is%x\n",tab[i]>>16);
		udelay(5);
		gpio_direction_output(dsp_hhwil,1);  //Hhwil low  
		//HPI_DATAwrite(tab[i]&0xffff);
		HPI_DATAwrite((unsigned short)tab[i+1]);
		printk("tab[i] is%x\n",tab[i]&0xffff);
	}
}

void hpi6416IncWrite(char *pval,int addr,int len)
{
	unsigned short temp;
	int i;	
	write6416HPIA(addr);	
	gpio_direction_output(dsp_hcntl1,1);  //HCNTL1 high /*set HCNTL1 high*/
	gpio_direction_output(dsp_hcntl0,0);  //HCNTL0 low /*set HCNTL0 LOW*/	
	i=0;
	while(i<len)
	{
		gpio_direction_output(dsp_hhwil,0);  //Hhwil low 		
		temp= *(unsigned short *)(pval+ (i<<1));		
		HPI_DATAwrite(temp);
		i=i+1;		
		gpio_direction_output(dsp_hhwil,1);  //Hhwil high		
		temp= *(unsigned short *)(pval+ (i<<1));		
		HPI_DATAwrite(temp);		
		i=i+1;
	}
}
#define DSP1_RST                0x55
#define DSP2_RST                0x56
#define DSP_INT                 0x57
#define FPGA_RESET              0x58
#define RAPIDIO_start_flag      0x59
/*******************************************************************************************************************
*
*
*
*******************************************************************************************************************/
static long hpi_load_ioctl(struct file *file, unsigned int cmd, int arg)
{
	switch (cmd) 
	{
	case  DSP1_RST: 
	{
		gpio_direction_output(dsp1_rst_gpio,1);
		gpio_set_value(dsp1_rst_gpio, 1);
	
		printk("dsp1 rst pull high\n");
		msleep(100);
		gpio_set_value(dsp1_rst_gpio, 0);
		printk("dsp1 rst push low\n");
		msleep(100);
		gpio_set_value(dsp1_rst_gpio, 1);
		printk("dsp1 reset success\n");
		
	}
	break;
	case  DSP2_RST:
	{
		gpio_direction_output(dsp2_rst_gpio,1);
		gpio_set_value(dsp2_rst_gpio, 1);
		printk("dsp2 rst pull high\n");
		msleep(100);
		gpio_set_value(dsp2_rst_gpio, 0);
		printk("dsp2 rst push low\n");
		msleep(100);
		gpio_set_value(dsp2_rst_gpio, 1);
		printk("dsp2 reset success\n");
	}
	break; 		
	case  DSP_INT:
	{  
		wirte6416hpic(0x00020002);
		printk("write once dspInt\n");
	}
	break;	
	case  FPGA_RESET :
	{
		gpio_direction_output(fpga_reset,1);
		gpio_set_value(fpga_reset, 0);
		printk("fpga rst push low\n");
		msleep(10);
		gpio_set_value(fpga_reset, 1);
		printk("fpga reset success\n");
		msleep(10);
	}
	break; 
	case  RAPIDIO_start_flag :
	{
		ld_tongdao->rapidio_start = 0xefef;
	}
	break; 
	default:
		return -1; 
	break;
	}
	return 0;
}

static int sca_hpi_open(struct inode *inode, struct file *filp)
{
   printk("open 6416_hpi\n");	
   return 0;
}

unsigned char Reverse(unsigned char byte) 
{
	unsigned char origin,temp,retValue,i;
     
	origin = byte;

	retValue=0;
	for(i=0;i<8;i++)
	{
		temp = origin&0x1;
		retValue = retValue<<1;
		retValue+=temp;
		origin = origin>>1;
	}
	return retValue;
}
static int fpga_gpmc_write(char *buff, int length)
{
	unsigned short temp,i;
	for(i = 0; i < length; i=i+2){
		temp = buff[i+1]|(buff[i]<<8);
		tongdao[2]->data = temp;
	}
	return length;
}

static int gpmc_ld_config(unsigned short *buff)
{
	unsigned short tmp,len,flag,data_count;
	long i=0;
	unsigned short data_H,data_L;
//	printk("In writting \n");
	len = buff[0];
//	printk("len  = 0x%x\n", len);
//	printk("buff[0]  = 0x%x\n",buff[0]);
//	printk("buff[1]  = 0x%x\n",buff[1]);

//	printk("ld_tongdao->ld  = 0x%x\n",ld_tongdao->ld);
	ld_tongdao->ld = buff[1];
	ld_tongdao->length = len;
	if(len > 1024) {
		printk("data count beyond buff length\n");
		return 2;
	}
	len += 3;
	len >>= 2;//len
	/*
	do {
		flag = ld_tongdao->data_L;
		if(data_count == 300)
		{
			printk("write err\n");
			return 0;
		}
		udelay(10);
		data_count++;
	}
	*/
	while(flag != 0x00);
//	printk("ld_tongdao->length  = 0x%x\n",ld_tongdao->length);
	for(i = 1; i < len; i ++) {
		data_L = buff[2*i];
		data_H = buff[2*i+1];
		ld_tongdao->data_H = data_H;
		ld_tongdao->data_L = data_L;
	}		
	ld_tongdao->stop_flag = 0xefef;
	return(len*4+2);
}
/*******************************************************************************************************************
*
*
*
*******************************************************************************************************************/
unsigned short select_buff[1];
static ssize_t  sca_hpi_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{  
	int i,fpga_count, ld_config_count;
	unsigned short select;
//	printk("-----------------------in writting-----------------------------------\n");
	if(copy_from_user(select_buff, buf, 2)) {
		printk("-------------error in copy from user----------------\n");		
	}
	select = select_buff[0];
//	printk("select = %d",select);
	if(select == 0x01){
		if (down_interruptible(&dsp[0].sem))
		return -1;
		hpi_base = (unsigned short*)tongdao[0];
		count = count-2;
//		printk("count = %d\n",count);
		dsp[0].hpi_buff=kmalloc(count,GFP_ATOMIC);
		if(copy_from_user(dsp[0].hpi_buff, buf+2, count*sizeof(char)))
		{
			printk("-------------error in copy from user----------------\n");		
		}		
		hpi6416AutoWrite(dsp[0].hpi_buff, count, 1024*dsp[0].write_count_dsp1);
		dsp[0].write_count_dsp1++;
		// i clear to zero
		if(count != 1024)
		dsp[0].write_count_dsp1 = 0;
		up(&dsp[0].sem);
		return count;
	}
	if(select == 0x02){
		if (down_interruptible(&dsp[1].sem))
		return -1;
		hpi_base = (unsigned short*)tongdao[1];
		dsp[1].write_count_dsp2 = 0;		
		if(count >1026)
		count = count-2;	
		dsp[1].hpi_buff=kmalloc(count,GFP_ATOMIC);
		if(copy_from_user(dsp[1].hpi_buff, buf+2, count*sizeof(char)))
		{
			printk("-------------error in copy from user----------------\n");		
		}		
		hpi6416AutoWrite(dsp[1].hpi_buff, count, 1024*dsp[1].write_count_dsp2);
		dsp[1].write_count_dsp2++;
		// i clear to zero
		if(count != 1024)
		dsp[1].write_count_dsp2 = 0;
		up(&dsp[1].sem);
		return count;
	}
	if(select == 0x03){
		if (down_interruptible(&fpga.sem))
		return -1;
		count = count -2;
//		printk("fpga_count = %d\n",count);	
		fpga.hpi_buff=kmalloc(count,GFP_ATOMIC);
		if(copy_from_user(fpga.hpi_buff, buf+2, count*sizeof(char))){
			printk("---------------error in copy from user-----------------\n");		
		}
		for(i = 0; i < count; i++){
//			printk("fpga.hpi_buff[%d] = %x\n",i,count);
			fpga.hpi_buff[i] = Reverse(fpga.hpi_buff[i]);
		}		
		fpga_count = fpga_gpmc_write(fpga.hpi_buff, count);
		up(&fpga.sem);
		return fpga_count;
	}
	if(select == 0x04){
		if (down_interruptible(&ld_config.sem))
		return -1;
		count = count -2;
//		printk("fpga_count = %d\n",count);	
		fpga.ld_buff=kmalloc(count,GFP_ATOMIC);
		if(copy_from_user(fpga.ld_buff, buf+2, count*sizeof(char))){
			printk("---------------error in copy from user-----------------\n");		
		}
//		printk("count = 0x%d\n",count);
		/*
		for(i = 0; i < count/2; i++)
		{
			printk("fpga.ld_buff[%d] = 0x%x\n",i,fpga.ld_buff[i]);
		}
		*/		
		ld_config_count = gpmc_ld_config(fpga.ld_buff);
		up(&ld_config.sem);
		return ld_config_count;
	}

} 

//demand whether program is loaded success
static  unsigned int  ack_flag(void)
{
	unsigned short load_status;
	load_status =  tongdao[0]->flag;
//	printk("load_status = 0x %4x\n",load_status);
	return load_status;
}
static ssize_t sca_hpi_read(struct file *filp, char __user *buf, size_t size,
     loff_t *ppos)
{	
	fpga.read_buff = kmalloc(2,GFP_ATOMIC);
	fpga.read_buff[0] = ack_flag();
	printk("fpga.read_buff[0] = 0x %4x\n",fpga.read_buff[0]);
	if(copy_to_user(buf, fpga.read_buff, size)){
		printk("copy_to_user err!\n");
		return 3;
	}
	return size;
}

void gpmc_cs_write_reg(int cs, int idx, u32 val)
{
	void __iomem *reg_addr;

	reg_addr = gpmc_base + GPMC_CS0_OFFSET + (cs * GPMC_CS_SIZE) + idx;
	writel_relaxed(val, reg_addr);
}
static u32 gpmc_cs_read_reg(int cs, int idx)
{
	void __iomem *reg_addr;

	reg_addr = gpmc_base + GPMC_CS0_OFFSET + (cs * GPMC_CS_SIZE) + idx;
	return readl_relaxed(reg_addr);
}

void  gpmc_init_DSP(void)
{ 
	
	int i,ret = 0;
	gpmc_base = ioremap(GPMC_BASE_ADDR, SZ_4K);
	for(i = 0 ;i < 3; i++)
	{
		ret = gpmc_cs_read_reg(i, GPMC_CS_CONFIG7);
		printk("CS %d GPMC_CS_CONFIG7 = %x\n",i,ret);
		printk("CS %d GPMC_CS_CONFIG1 = %x\n",i,gpmc_cs_read_reg(i, GPMC_CS_CONFIG1));
	}
	gpmc_cs_write_reg(1, GPMC_CS_CONFIG7, 0xf41);
	gpmc_cs_write_reg(2, GPMC_CS_CONFIG7, 0xf42);
//	gpmc_cs_enable_mem(GPMC_CS);
	for(i = 0 ;i < 3; i++)
	{
		ret = gpmc_cs_read_reg(i, GPMC_CS_CONFIG7);
		printk("CS %d GPMC_CS_CONFIG7 = %x\n",i,ret);
	}	
	mem_base = 0x2002000;
	printk( "before request address = %lx\n", mem_base);
	/*
	ret = gpmc_cs_request(GPMC_CS,SZ_8K,&mem_base);
	printk("ret = %d\n",ret);	
	if(ret <0)
	{
		printk("Failed request for GPMC CS request\n");
		return -1;
	}

	printk( "Got CS = %d, address = %lx\n", GPMC_CS,mem_base);
//	gpmc_cs_disable_mem(GPMC_CS);
	*/
	if (!request_mem_region(mem_base , SZ_8K,  "6416_hpi_men")) {
		printk( "Request_mem_region failed.\n");
		gpmc_cs_free(GPMC_CS);
		return -1;
	}
	hpi_base = ioremap(mem_base, SZ_8K);
	//select base_address
	for(i = 0; i < 3; i++){
		tongdao[i] = (struct sca_hpi_driver *)(hpi_base+0x80+i*0x04);
		printk("tongdao[%d]=0x%x\n",i,(unsigned int)tongdao[i]);
	}
	ld_tongdao = (struct sca_hpi_driver *)(hpi_base+0x60);
	printk("ld_tongdao =0x%x\n",(unsigned int)ld_tongdao);
	printk("dsp base ipremap OK!\n"); 
}
/*******************************************************************************************************************
*
*gpio配置
*
*******************************************************************************************************************/  
void gpio_configure(void)
{
	if(gpio_request(dsp_hcntl0,"dsp_hcntl0\n")<0)
	{
	printk("failed to request gpio for dsp_hcntl0\n");
	}
	if(gpio_request(dsp_hcntl1,"dsp_hcntl1\n")<0)
	{
		printk("failed to request gpio for dsp_hcntl1 \n");
	}
	if(gpio_request(dsp_hhwil,"dsp_hhwil\n")<0)
	{
		printk("failed to request gpio for dsp_hhwil GPIO_TO_PIN(3,20);\n");
	}
	if(gpio_request(dsp_has,"dsp_hhwil\n")<0)
	{
		printk("failed to request gpio for dsp_hhwil GPIO_TO_PIN(3,20);\n");
	}
	if(gpio_request(dsp1_rst_gpio,"dsp1_rst_gpio\n")<0)
	{
		printk("failed to request gpio for dsp1_rst_gpio");
	}	
	if(gpio_request(dsp2_rst_gpio,"dsp2_rst_gpio\n")<0)
	{
		printk("failed to request gpio for dsp2_rst_gpio");
	}
		
	if(gpio_request(fpga_reset,"IO_change_status1\n")<0)
	{
		printk("failed to request gpio for fpga_reset");
	}
	
	gpio_direction_output(dsp_hcntl0,1);  //HCNTL0 low 
	gpio_direction_output(dsp_hcntl1,1);  //HCNTL0 low 
	gpio_direction_output(dsp_hhwil,1);  //HCNTL0 low 
	gpio_direction_output(dsp_has,1);  //HCNTL0 low 
	gpio_direction_output(fpga_reset,1);  //HCNTL0 low 
}
void clear_fifo(void)
{
	dsp[0].write_count_dsp1 = 0;
	dsp[1].write_count_dsp2 = 0;
}
static struct file_operations sca_hpi_fops = {
	.owner =             THIS_MODULE,
	.read  =             sca_hpi_read,
	.write =             sca_hpi_write,
	.open =              sca_hpi_open,
	.unlocked_ioctl =    hpi_load_ioctl,
};

void semaphore_init(void)
{
	sema_init(&dsp[0].sem, 1);
	sema_init(&dsp[1].sem, 1);
	sema_init(&dsp[2].sem, 1);
	sema_init(&fpga.sem, 1);
	sema_init(&ld_config.sem, 1);
}
/*******************************************************************************************************************
*
*
*
*******************************************************************************************************************/
static int __init sca_hpi_init_module(void)
{
	int result=0;
	dev_t dev = 0;
	dev = MKDEV(sca_hpi_major,sca_hpi_minor); 
	printk("sca_hpi_init_module!\n");      
	result = register_chrdev_region(dev, 1, DEVICE_NAME);	
	if (result < 0)
	{
		printk(KERN_WARNING "sca_arm_fpga: can't get major %d\n", sca_hpi_major);
	}  
	else 
	{
		printk("sca_hpi register chrdev ok!\n");
	}
	sca_hpi_class=class_create(THIS_MODULE,DEVICE_NAME);
	
	printk("class_create  init ok!\n");	
	device_create(sca_hpi_class,NULL,dev,NULL,DEVICE_NAME);
	printk("device_create  init ok!\n");
	cdev_init(&sca_hpi_Cdev,&sca_hpi_fops);//初始化cdev使得结构与文件结构体相关联
	sca_hpi_Cdev.ops=&sca_hpi_fops;//注册字符设备
	if(cdev_add(&sca_hpi_Cdev,dev,1))
	{ 
	printk(KERN_WARNING "cdev_add\n");
	}  
	printk("gpio_configure init start!\n");
	gpio_configure();
	printk("gpio_configure init ok!\n");
	semaphore_init();
	gpmc_init_DSP();
	clear_fifo();
	printk("sca_hpi_init driver!\n");
	return 0;
}
module_init(sca_hpi_init_module);

void release_gpio(void)
{
	gpio_free(dsp_hcntl0);
	gpio_free(dsp_hcntl1);
	gpio_free(dsp_hhwil);
	gpio_free(dsp_has);	
	gpio_free(dsp1_rst_gpio);
	gpio_free(dsp2_rst_gpio);
	gpio_free(fpga_reset);
}
void free_buff(void)
{
	printk("before kfree\n");
	if(dsp[0].hpi_buff)
	{
	kfree(dsp[0].hpi_buff);
	}
	if(dsp[1].hpi_buff)
	{
	kfree(dsp[1].hpi_buff);
	}
	if(fpga.hpi_buff)
	{
	kfree(fpga.hpi_buff);
	}
	if(fpga.ld_buff)
	{
	kfree(fpga.ld_buff);
	}
	printk("after kfree\n");
}
static void sca_hpi_cleanup_module(void)
{
	printk( " in exit mem_base address = %lx\n", mem_base);
	dev_t devno = MKDEV(sca_hpi_major,sca_hpi_minor);
	device_destroy(sca_hpi_class, devno);
	class_destroy(sca_hpi_class);
	cdev_del(&sca_hpi_Cdev);
	unregister_chrdev_region(devno, 1);
	iounmap(gpmc_base);
	iounmap(hpi_base); 
	release_mem_region(mem_base, SZ_8K);
 
	release_gpio();
	free_buff();
	printk("exit dsp_hpi success!!!\n");
}   
module_exit(sca_hpi_cleanup_module);
MODULE_AUTHOR("chenheng <411510260@qq.com>");
MODULE_DESCRIPTION("Common am3358  hpi driver for 2183#");
MODULE_LICENSE("GPL");


 


 
