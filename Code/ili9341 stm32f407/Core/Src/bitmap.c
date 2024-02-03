//#include "fatfs.h"
//#include "string.h"
//#include "bitmap.h"
//#include "ili9341.h"
//#include "string.h"
//#include "stdio.h"
//extern SPI_HandleTypeDef hspi2;
//
//FRESULT LSBFirstWrite(FIL *fp,  DWORD value, uint8_t len)
//{
//	UINT bw;
//	char lsb[len];
//	for (int i = 0; i < len; i++)
//	{
//		lsb[i] = (value>>(i*8))&0xFF;
//	}
//
//	return f_write(fp, lsb, len, &bw);
//
//}
//
//FRESULT saveScreenBigmap(FIL *fp)
//{
//	FRESULT res;
//
//	uint8_t RGBMask[12] = {0x00,0xf8,0x00,0x00,0xe0,0x07,0x00,0x00,0x1f,0x00,0x00,0x00};
//	UINT bw;
//	DWORD tempValue;
//	uint16_t w,h;
//	w=lcdGetWidth();
//	h=lcdGetHeight();
//	uint16_t pixvalue;
//	char buff[w*2];
//
//	//write file header
//	// type: BM
//	res = f_write(fp, "BM", 2, &bw); if (res!=FR_OK) return res;
//	// size: width*height*2(bytes,16bits)+14(file header)+40(info header)+12(RGB mask)
//	tempValue = lcdGetWidth()*lcdGetHeight()*2+66;
//	res = LSBFirstWrite(fp, tempValue, 4); if (res!=FR_OK) return res;
//	// Reserved1, Reserved2: 0,0
//	memset(buff,0,4);
//	res = f_write(fp, buff, 4, &bw); if (res!=FR_OK) return res;
//	// bitmap offset: 14+40+12=66
//	res = LSBFirstWrite(fp, 66, 4); if (res!=FR_OK) return res;
//	// File Information Header
//	// info header size:40
//	res = LSBFirstWrite(fp, 40, 4); if (res!=FR_OK) return res;
//	//width: lcd width
//	res = LSBFirstWrite(fp, (DWORD)w, 4); if (res!=FR_OK) return res;
//	//height: lcd height
//	res = LSBFirstWrite(fp, (DWORD)h, 4); if (res!=FR_OK) return res;
//	//Planes:1
//	res = LSBFirstWrite(fp, 1, 2); if (res!=FR_OK) return res;
//	//bitCount:16 RGB565
//	res = LSBFirstWrite(fp, 16, 2); if (res!=FR_OK) return res;
//	//Compression:BI_BITFIELDS=3,
//	res = LSBFirstWrite(fp, 3, 4); if (res!=FR_OK) return res;
//	//sizeImage:0, do not care
//	res = LSBFirstWrite(fp, tempValue, 4); if (res!=FR_OK) return res;
//	//x: 72 dpi=2835 PelsPerMeter
//	res = LSBFirstWrite(fp, 2835, 4); if (res!=FR_OK) return res;
//	//y: 72 dpi=2835 PelsPerMeter
//	res = LSBFirstWrite(fp, 2835, 4); if (res!=FR_OK) return res;
//	//color use:0
//	res = LSBFirstWrite(fp, 0, 4); if (res!=FR_OK) return res;
//	//color important:0
//	res = LSBFirstWrite(fp, 0, 4); if (res!=FR_OK) return res;
//	// RGB565 mask
//	res = f_write(fp, RGBMask, 12, &bw); if (res!=FR_OK) return res;
//
//	//write image pixel from LB to RT
//	for (int y = h-1; y >= 0; y--)
//	{
//		memset(buff,0, w*2);
//		for (int x = 0; x <=w-1; x++)
//		{
//			pixvalue=0;
//			pixvalue = lcdReadPixel((uint16_t)x, (uint16_t)y);
//			buff[x*2+1] = ((pixvalue>>8) & 0xFF);
//			buff[x*2] = (pixvalue & 0xFF);
//
//		}
//		res = f_write(fp, buff,w*2,&bw); if (res!=FR_OK) return res;
//	}
//
//
//	return res;
//
//
//}
//
//uint8_t saveScreen(char* path)
//{
//	FATFS fs;
//	FIL file;
//	FRESULT res;
//
//	char fn[256];
//
//
//
//	res = f_mount(&fs, path, 1);
//	if (res==FR_OK)
//	{
//		sprintf(fn,"%sScreen%ld.bmp",path,get_fattime()&0xFFFF);
//		res = f_open(&file, fn, FA_CREATE_ALWAYS|FA_WRITE);
//		if (res==FR_OK)
//		{
//			lcdSetWindow(0, 0, lcdGetWidth()-1, lcdGetHeight()-1);
//			res=saveScreenBigmap(&file);
//			f_close(&file);
//		}
//		f_mount(&fs, "", 0);
//	}
//
//
//
//
//	return res;
//
//}
//
//FRESULT bitmapHeader(FIL* fp, BITMAPFILEHEADER *fh, BITMAPINFOHEADER *ih)
//{
//	FRESULT r;
//	uint8_t fbuff[14], ibuff[40],maskbuf[12];
//	UINT br;
//	if ((r=f_read(fp, fbuff, 14, &br)) != FR_OK) return r;
//	fh->bfType=fbuff[0] << 8 | fbuff[1];
//	fh->bfSize = fbuff[5] << 24 | fbuff[4] << 16 |fbuff[3] << 8 | fbuff[2];
//	fh->bfReserved1 =fbuff[7] << 8 |fbuff[6];
//	fh->bfReserved2 =fbuff[9] << 8 |fbuff[8];
//	fh->bfOffBits = fbuff[13] << 24 | fbuff[12] << 16 |fbuff[11] << 8 | fbuff[10];
//
//	if ((r=f_read(fp, ibuff, 40, &br)) != FR_OK) return r;
//	ih->biSize = ibuff[3] << 24 | ibuff[2] << 16 |ibuff[1] << 8 | ibuff[0];
//	ih->biWidth = ibuff[7] << 24 | ibuff[6] << 16 |ibuff[5] << 8 | ibuff[4];
//	ih->biHeight = ibuff[11] << 24 | ibuff[10] << 16 |ibuff[9] << 8 | ibuff[8];
//	ih->biPlanes = ibuff[13] << 8 | ibuff[12];
//	ih->biBitCount = ibuff[15] << 8 | ibuff[14];
//	ih->biCompression = ibuff[19] << 24 | ibuff[18] << 16 |ibuff[17] << 8 | ibuff[16];
//	ih->biSizeImage =ibuff[23] << 24 | ibuff[22] << 16 |ibuff[21] << 8 | ibuff[20];
//	ih->biXPelsPerMeter = ibuff[27] << 24 | ibuff[26] << 16 |ibuff[25] << 8 | ibuff[24];
//	ih->biYPelsPerMeter =ibuff[31] << 24 | ibuff[30] << 16 |ibuff[29] << 8 | ibuff[28];
//	ih->biClrUsed = ibuff[35] << 24 | ibuff[34] << 16 |ibuff[33] << 8 | ibuff[32];
//	ih->biClrImportant = ibuff[39] << 24 | ibuff[38] << 16 |ibuff[37] << 8 | ibuff[36];
//
//	if ((r=f_read(fp, maskbuf, 12, &br)) != FR_OK) return r;
//
//	f_close(fp);
//
//	return FR_OK;
//}
