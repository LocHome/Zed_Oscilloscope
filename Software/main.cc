/*
 * Empty C++ Application
 */

/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/


/***************************** Include Files *********************************/
#include "math.h"
#include <complex>
#include <stdio.h>
#include "xaxidma.h"
#include "xil_io.h"
#include "xsysmon.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xuartlite.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "xgpio.h"
#include "xscugic.h"
#include "fft.h"
#include "xadcps.h"


using namespace std;
/************************** Constant Definitions *****************************/

#define UART1_DEVICE_ID				XPAR_XUARTPS_0_DEVICE_ID
#define UART1_BAUDRATE				(u32) 230400

#define UARTLITE0_DEVICE_ID			XPAR_UARTLITE_0_DEVICE_ID

#define DMA_DEV_ID					XPAR_AXIDMA_0_DEVICE_ID
#define DDR_BASE_ADDR				XPAR_AXIDMA_0_BASEADDR

#define MAX_PKT_LEN					(int) 4096

#define XADC_DEVICE_ID				XPAR_SYSMON_0_DEVICE_ID

#define XADC_FIFO_ADAPTER_ID		XPAR_XADC_AXIS_FIFO_ADAPTER_0_BASEADDR

#define MEM_BASE_ADDR 				XPAR_PS7_RAM_0_S_AXI_BASEADDR
#define TX_BUFFER_BASE 				(MEM_BASE_ADDR)
#define RX_BUFFER_BASE 				(MEM_BASE_ADDR + 0x00001000)

#define GPIO_DEVICE_ID  			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_CHANNEL				1

#define RX_INTR_ID					XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR
#define INTC_DEVICE_ID 				XPAR_SCUGIC_SINGLE_DEVICE_ID

#define NFFT_POWER					(int) 12
#define DATA_POINTS					(int) 4096		// (i.e. powf(2,NFFT_POWER)
#define NFFT_POINTS					(int) 4096		// (i.e. powf(2,NFFT_POWER)

#define UARTLITE_FIFOLEN			(int) 16
#define UARTLITE_RX_PACKET_LEN		(u8)   6

#define UARTLITE_RX_PREAMBLE		(u8)  0x02
#define UARTLITE_RX_PID_WAVEFORM	(u8)  0xF8
#define UARTLITE_RX_PID_SPECTRUM	(u8)  0xF7
#define UARTLITE_RX_POSTAMBLE       (u8)  0x03

/**************************** Type Definitions *******************************/

/************************** Variable Definitions *****************************/
static XUartLite 		UartLiteInst;
static XUartPs 			Uart1Inst;
static XSysMon 			xADCInst;
static XAxiDma 			AxiDmaInst;
static XGpio			xGPIOInst;
static XScuGic 			INTCInst;

complex<float> xadcSamples[NFFT_POINTS];
complex<float> twiddleTable[NFFT_POWER];
signed char fftmagdB[NFFT_POINTS];

// create a handle to FFTc object
FFTc* m_FFTc;

u16 adcOutBuff[MAX_PKT_LEN];
u8 RecvBuffer[UARTLITE_RX_PACKET_LEN];
u8 DMA_DRDY = 0;


/************************** Function Prototypes ******************************/
int HardwareBlockDesignInit(void);
int UARTInit(XUartPs *pUARTx,u16 DeviceID, u32 Baudrate);
int UartLiteInit(u16 DeviceId, XUartLite *pUartLiteInst);
int AxiDMAInit(u16 DeviceId,XAxiDma *pAxiDmaInst);
int xADCInit(u16 DeviceId,XSysMon *pxADCInst);
int xGPIOInit(XGpio *pGPIOInst, u16 DeviceId);
int SetupIntrSystem(XScuGic *pINTCInst, u16 DeviceID);
static void s2mm_isr(void* pCallBack);

void runSignalScopeApp(void);
int waitForCommands(u16 DeviceId, XUartLite *pUartLiteInst);
int getXADCFrameOfSamples(u16 adcDeviceID, XSysMon *pxADCInst, u16 dmaDeviceID, XAxiDma *pAxiDmaInst);
int uart0Lite_Send(u16 DeviceId, XUartLite *pUartLiteInst, void *pRS232_txBuff, int numOfBytestoSend);
void sendReply(u8 *pRxBuffer);
void sendSpectrum(void);
void sendWaveform(void);

/*****************************************************************************
*
* Main function
*
* @param	None.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int main(void)
{
	int Status;

//	 Section I.  Initialized the components in the hardware block design.
	Status = HardwareBlockDesignInit();
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

//  II.  Launch and run our application.
	runSignalScopeApp();


	return XST_SUCCESS;
}

/******************************************************************************
 * Hardware Block Design Component Init
 *
 *
 *****************************************************************************/
int HardwareBlockDesignInit(void)
{
	int Status;

	// Initialize GPIO pins
	Status = xGPIOInit(&xGPIOInst, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Initialize UART1 (UART to USB)
	Status = UARTInit(&Uart1Inst,UART1_DEVICE_ID,UART1_BAUDRATE);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Configure UARTLITE Device
	Status = UartLiteInit(UARTLITE0_DEVICE_ID,&UartLiteInst);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Setup interrupt for DMA's s2mm transfer (stream to memory mapped transfer)
	Status = SetupIntrSystem(&INTCInst,INTC_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Initialize AXI DMA
	Status = AxiDMAInit(DMA_DEV_ID,&AxiDmaInst);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Initialize/configure xADC
	Status = xADCInit(XADC_DEVICE_ID,&xADCInst);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/******************************************************************************************
 * Main Application
 *
 *
 *****************************************************************************************/
void runSignalScopeApp(void)
{
	// Get an instance and intialized FFTc
	m_FFTc = FFTc::GetInstance();
	m_FFTc->Setup(&twiddleTable[0], NFFT_POWER);


	while (1)
	{
		// Get a "Frame" of XADC data and calculate FFT
		getXADCFrameOfSamples(XADC_DEVICE_ID, &xADCInst, DMA_DEV_ID, &AxiDmaInst);
		// wait for commands to send data
		waitForCommands(UARTLITE0_DEVICE_ID,&UartLiteInst);
		// send a reply
		sendReply(&RecvBuffer[0]);
	}
}


/******************************************************************************************
 * Initialize GPIO pin.
 * User a GPIO pin as output pin for debugging and code execution time measurement
 *
 ******************************************************************************************/
int xGPIOInit(XGpio *pGPIOInst, u16 DeviceId)
{
	XGpio_Config *Config;
	int Status;

	Config = XGpio_LookupConfig(DeviceId);
	if (Config == NULL)
	{
		xil_printf("ERROR! No hardware configuration found for XGPIO device id: %d.\r\n", DeviceId);
		return XST_FAILURE;
	}

	/* GPIO driver initialisation */
	Status = XGpio_Initialize(pGPIOInst, DeviceId);
	if (Status != XST_SUCCESS)
	{
		xil_printf("ERROR! Initialization of XGPIO failed.  Status: %d.\r\n", Status);
		return XST_FAILURE;
	}

//	Set the direction for the LEDs to output.
	XGpio_SetDataDirection(pGPIOInst, GPIO_CHANNEL, 0x0);		// set data direction to '0' for output and '1' for input.
	xil_printf("XGPIO configured for outputs. \r\n");

//	Drive the ouput pins to logic '0'
	XGpio_DiscreteWrite(pGPIOInst, GPIO_CHANNEL, 0x0);
	xil_printf("XGPIO outputs set to 0 \r\n");

	return XST_SUCCESS;
}

/*****************************************************************************
* Initialize UART1.
* This is the UART-to-USB and it is used to send debug messages to a terminal window.
*
******************************************************************************/
int UARTInit(XUartPs *pUARTx,u16 DeviceID, u32 Baudrate)
{
	XUartPs_Config *Config;
	int Status;

	Config = XUartPs_LookupConfig(DeviceID);
	if (Config == NULL)
	{
		xil_printf("ERROR! No hardware configuration found for UART1 (i.e. UART to USB) device id: %d.\r\n", DeviceID);
		return XST_FAILURE;
	}
	Status = XUartPs_CfgInitialize(pUARTx,Config,Config->BaseAddress);
	if (Status != XST_SUCCESS)
	{
		xil_printf("ERROR! Initialization of UART1 failed.  Status: %d.\r\n", Status);
		return XST_FAILURE;
	}

	Status = XUartPs_SelfTest(pUARTx);
	if (Status != XST_SUCCESS)
	{
		xil_printf("ERROR! Self test failed.  UART1 Device is not working! Status: %d.\r\n", Status);
		return XST_FAILURE;
	}

	// Now set the uart baudrate to the baudrate we want.
	XUartPs_SetBaudRate(pUARTx, Baudrate);
	xil_printf("UART1 is working.  Baudrate is set to %d.\r\n", Baudrate);

	return XST_SUCCESS;
}

/****************************************************************************************************************
 * Initialize a UARTLITE
 * We've configured this UARTLITE to run a very fast baudrate of 921.600 Kbps.
 * This UARTLITE will be used to send "xADC" raw and post processed (i.e. FFT) data from the zynq to an application running
 * on a PC to display the data.
 ****************************************************************************************************************/
int UartLiteInit(u16 DeviceId, XUartLite *pUartLiteInst)
{
	XUartLite_Config *Config;
	int Status;

	Config = XUartLite_LookupConfig(DeviceId);
	if (Config == NULL)
	{
		xil_printf("ERROR! No hardware configuration found for UartLitex device id: %d.\r\n", DeviceId);
		return XST_FAILURE;
	}

	Status = XUartLite_CfgInitialize(pUartLiteInst,Config,Config->RegBaseAddr);
	if (Status != XST_SUCCESS)
	{
		xil_printf("ERROR! Initialization of UartLitex failed.  Status: %d.\r\n", Status);
		return XST_FAILURE;
	}

	// Cannot set baudrate in software for UartLite (i.e. UartLite IP does not have this capability.)
	// Must set baudrate in hardware (i.e. in Vivado).
	xil_printf("UartLite baudrate must be set in hardware (i.e. in Vivado). \r\n");

	// do self test
	Status = XUartLite_SelfTest(pUartLiteInst);
	if (Status != XST_SUCCESS)
	{
		xil_printf("UartLitex is working.  Baudrate is set in the hardware Block Design.\r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************
* Setup Interrupt, s2mm (stream to memory mapped), so we don't have to use
* polling to determine when we've received a frame of data (4096 words).
*
******************************************************************************/
int SetupIntrSystem(XScuGic *pINTCInst, u16 DeviceID)
{
	int             status = 0;
	XScuGic_Config* cfg_ptr;

	cfg_ptr = XScuGic_LookupConfig(DeviceID);
	if (!cfg_ptr)
	{
		xil_printf("ERROR! No hardware configuration found for Interrupt Controller device id: %d.\r\n", DeviceID);
		return XST_FAILURE;
	}

	// Initialize driver
	status = XScuGic_CfgInitialize(&INTCInst, cfg_ptr, cfg_ptr->CpuBaseAddress);
	if (status != XST_SUCCESS)
	{
		xil_printf("ERROR! Initialization of Interrupt Controller failed.  Status: %d.\r\n", status);
		return XST_FAILURE;
	}

	// Set interrupt priorities and trigger type
	XScuGic_SetPriorityTriggerType(&INTCInst, RX_INTR_ID, 0xA0, 0x3);

	// Connect handlers
	status = XScuGic_Connect(&INTCInst, RX_INTR_ID, (Xil_InterruptHandler)s2mm_isr, &AxiDmaInst);
	if (status != XST_SUCCESS)
	{
		xil_printf("ERROR! Unable to connect s2mm_isr to the interrupt controller. Status: %d\r\n", status);
		return XST_FAILURE;
	}

	// Enable interrupt.
	XScuGic_Enable(&INTCInst, RX_INTR_ID);

	// Initialize exception table and register the interrupt controller handler with exception table
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &INTCInst);

	// Enable non-critical exceptions
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/*****************************************************************************************************************
 * Initialize AXI DMA to move a stream of data from XADC to
 * Memory.
 *
 *****************************************************************************************************************/
int AxiDMAInit(u16 DeviceId,XAxiDma *pAxiDmaInst)
{
	int Status;
	XAxiDma_Config *CfgPtr;
	int reset_done;

    CfgPtr = XAxiDma_LookupConfig(DeviceId);
    if (!CfgPtr) {
    	printf("No config found for AXI DMA device id: %d\r\n", DeviceId);
    	return XST_FAILURE;
    }

    Status = XAxiDma_CfgInitialize(pAxiDmaInst, CfgPtr);
    if (Status != XST_SUCCESS) {
    	printf("AXI DMA Initialization failed. Status: %d\r\n", Status);
    	return XST_FAILURE;
    }

	XAxiDma_Reset(pAxiDmaInst);
	while (!XAxiDma_ResetIsDone(pAxiDmaInst)) {}

	// Enable interrupt for s2mm (i.e. device to DMA)
	XAxiDma_IntrEnable(pAxiDmaInst, (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK), XAXIDMA_DEVICE_TO_DMA);

    XAxiDma_Reset(pAxiDmaInst);
    reset_done = XAxiDma_ResetIsDone(pAxiDmaInst);
    while(reset_done != 1)
    {
    	// can't reset axi dma.
    }

    printf("AXI DMA Initialized!!! \r\n");

    return XST_SUCCESS;
}

/****************************************************************************************************************
 * Initialized XADC.
 * Running XADC in single channel mode and sampling at 1 Msps.
 ****************************************************************************************************************/
int xADCInit(u16 DeviceId,XSysMon *pxADCInst)
{
	int Status;
	XSysMon_Config *CfgPtr;

	Xil_Out32(XADC_FIFO_ADAPTER_ID,MAX_PKT_LEN);

	CfgPtr = XSysMon_LookupConfig(DeviceId);
    if (CfgPtr == NULL)
    {
    	printf("No Config found for xADC device ID: %d\r\n", DeviceId);
        return XST_FAILURE;
    }

    Status = XSysMon_CfgInitialize(pxADCInst, CfgPtr, CfgPtr->BaseAddress);
    if (Status != XST_SUCCESS)
    {
    	printf("XADC Initialization failed. Status: %d \r\n", Status);
    	return XST_FAILURE;
    }

    // Set xadc to single channel mode and select channel aux 14 as input
    XSysMon_SetSequencerMode(pxADCInst, XSM_SEQ_MODE_SINGCHAN);
    XSysMon_SetSingleChParams(pxADCInst, XSM_CH_AUX_MIN,FALSE, FALSE, FALSE);

    printf("xADC Initialized!!!! \r\n");

    return XST_SUCCESS;
}

/*************************************************************************************
 * Interrupt Service Routine for s2mm
 *
 *************************************************************************************/
static void s2mm_isr(void* pCallBack)
{
	int Index;
	u32 IRQStatus;
	XAxiDma* pDMAInst = (XAxiDma*)pCallBack;
	int	numberOfTransfers;

	// Disable interrupt
	XAxiDma_IntrDisable(pDMAInst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	// get IRQ status
	IRQStatus = XAxiDma_IntrGetIrq(pDMAInst, XAXIDMA_DEVICE_TO_DMA);

	// Acknowledge pending interrupts
	XAxiDma_IntrAckIrq(pDMAInst, IRQStatus, XAXIDMA_DEVICE_TO_DMA);

	// If no interrupt is asserted, we do not do anything
	if (!(IRQStatus & XAXIDMA_IRQ_ALL_MASK))
		return;

	// Raised the output pin to logic '1'
	// This pin is toggled over these codes/tasks.
	// Hook up this pin to logic analyzer or oscilloscope to measure how long of execution time
	// the following codes take.
	XGpio_DiscreteWrite(&xGPIOInst, GPIO_CHANNEL, 0x1);
	numberOfTransfers = sizeof(adcOutBuff)/sizeof(adcOutBuff[0]);
    for (Index=0;Index<numberOfTransfers;Index++)
    {
    	// Fill the time sample buffer (real value)
    	adcOutBuff[Index] = Xil_In16(RX_BUFFER_BASE+Index*sizeof(adcOutBuff[0]));
    	// Fill the FFT sample buffer (complex value with complex component set to 0.
    	xadcSamples[Index] = complex<float>(XAdcPs_RawToVoltage(adcOutBuff[Index]),0.0);
    }
    // Calculate FFT.
    m_FFTc->FFT(&xadcSamples[0], &twiddleTable[0], NFFT_POWER, NFFT_POINTS);
    // Convert FFT complex output to magnitude^2 to dB.
    m_FFTc->ConvertComplex2MagnitudedB(&xadcSamples[0],&fftmagdB[0],NFFT_POINTS);

    //Lower the ouput pin to logic '0'
	XGpio_DiscreteWrite(&xGPIOInst, GPIO_CHANNEL, 0x0);

	// set this flag to exit the while loop in "getXADCFrameOfSamples"
	DMA_DRDY = 1;

	return;
}

/***************************************************************************************************************
 * Get a frame of xADC data samples (i.e. MAX_PKT_LEN words)
 *
 ***************************************************************************************************************/
int getXADCFrameOfSamples(u16 adcDeviceID, XSysMon *pxADCInst, u16 dmaDeviceID, XAxiDma *pAxiDmaInst)
{
	int reset_done;
	int Status;
	int numOfBytesToTransfer;
	u8 numLoops;

    XAxiDma_IntrEnable(pAxiDmaInst, (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK), XAXIDMA_DEVICE_TO_DMA);
    numOfBytesToTransfer = MAX_PKT_LEN*sizeof(adcOutBuff[0]);
    Status = XAxiDma_SimpleTransfer(pAxiDmaInst,(u32) RX_BUFFER_BASE,numOfBytesToTransfer, XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS)
    {
    	printf("DMA transfer failed.  Status %d\r\n", Status);
    	return XST_FAILURE;

    }

    DMA_DRDY = 0;
    while(DMA_DRDY == 0)
    {
    	// Long wait here.
    	// If you have a RTOS running then
    	// run another tasks (i.e. waiting for semaphore in here instead of waiting for DMA_DRDY get set
    }


    return XST_SUCCESS;
}

/******************************************************************************************
 *  Wait for a command from the application running on PC.
 *  The PC app will either asked for raw xADC data samples or FFT spectrum of the xADC data.
 ******************************************************************************************/
int waitForCommands(u16 DeviceId, XUartLite *pUartLiteInst)
{
	unsigned int ReceivedCount = 0;

	while(1)
	{
		ReceivedCount += XUartLite_Recv(pUartLiteInst,RecvBuffer+ReceivedCount,UARTLITE_RX_PACKET_LEN-ReceivedCount);
		if (ReceivedCount == UARTLITE_RX_PACKET_LEN)
		{
			break;
		}
	}

	return XST_SUCCESS;
}

/********************************************************************************************
 * Check the request from the app running on the PC and reply with the appropriate data set.
 *
 ********************************************************************************************/
void sendReply(u8 *pRxBuffer)
{
	u8 checksum = 0;
	int i;

	for(i=0;i<UARTLITE_RX_PACKET_LEN-2;i++)
		checksum = checksum + pRxBuffer[i];

	checksum = checksum & 0xFF;
	if ((pRxBuffer[0] != UARTLITE_RX_PREAMBLE) || (pRxBuffer[4]!=checksum) || (pRxBuffer[5] != UARTLITE_RX_POSTAMBLE))
	{
		printf("Received Bad/Corrupted packet!!! \r\n");
		return;
	}

	if (pRxBuffer[1] == UARTLITE_RX_PID_SPECTRUM)
		sendSpectrum();
	else if (RecvBuffer[1] == UARTLITE_RX_PID_WAVEFORM)
		sendWaveform();
	else
	{
		printf("Received Bad Command!!! \r\n");
		return;
	}

	return;
}

/*****************************************************************************************************
 * Reply with spectrum data
 *
 ****************************************************************************************************/
void sendSpectrum(void)
{
	int txedByteCount = 0;
	signed char *pSpectrumBuffer;
	int BytesTx;
	int Status;

	pSpectrumBuffer = &fftmagdB[0];
	txedByteCount = 0;
	while(txedByteCount < (int)sizeof(fftmagdB))
	{
		BytesTx = (txedByteCount+UARTLITE_FIFOLEN) <= (int)sizeof(fftmagdB) ? UARTLITE_FIFOLEN:(int)sizeof(fftmagdB)-txedByteCount;
		Status = uart0Lite_Send(UARTLITE0_DEVICE_ID, &UartLiteInst,pSpectrumBuffer,BytesTx);
		txedByteCount += BytesTx;
		pSpectrumBuffer += UARTLITE_FIFOLEN/sizeof(fftmagdB[0]); // increment pointer by word size.
	}

	return;
}

/*****************************************************************************************************
 * Reply with raw xADC data.
 *
 *****************************************************************************************************/
void sendWaveform(void)
{
	int txedByteCount = 0;
	u16 *pdataBuffer;
	int BytesTx;
	int Status;

	pdataBuffer = &adcOutBuff[0];
	txedByteCount = 0;
	while(txedByteCount < (int)sizeof(adcOutBuff))
	{
		BytesTx = (txedByteCount+UARTLITE_FIFOLEN) <= (int)sizeof(adcOutBuff) ? UARTLITE_FIFOLEN:(int)sizeof(adcOutBuff)-txedByteCount;
		Status = uart0Lite_Send(UARTLITE0_DEVICE_ID, &UartLiteInst,pdataBuffer,BytesTx);
		txedByteCount += BytesTx;
		pdataBuffer += UARTLITE_FIFOLEN/(int)sizeof(adcOutBuff[0]); // increment pointer by word size.
	}

	return;
}

/******************************************************************************************************
 * Sending response.
 *
 ******************************************************************************************************/
int uart0Lite_Send(u16 DeviceId, XUartLite *pUartLiteInst, void *pRS232_txBuff, int numOfBytestoSend)
{
	int SentCount=0;


	SentCount = XUartLite_Send(pUartLiteInst, (u8 *)pRS232_txBuff, numOfBytestoSend);
	if (SentCount != numOfBytestoSend)
	{
		return XST_FAILURE;
	}

	while ((bool)XUartLite_IsSending(pUartLiteInst) == true)
	{
		// Keep looping until all data is sent.
	}


	return XST_SUCCESS;
}




