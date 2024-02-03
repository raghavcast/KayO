//enum compressionMethod {
//	BI_RGB=0,
//	BI_RLE8=1,
//	BI_RLE4=2,
//	BI_BITFIELDS=3,
//	BI_JPEG=4,
//	BI_PNG=5,
//	BI_ALPHABITFIELDS=6
//};
//typedef struct tagBITMAPFILEHEADER
//{
//	uint16_t 	bfType;
//	DWORD 		bfSize;	// file size
//	uint16_t 	bfReserved1;
//	uint16_t 	bfReserved2;
//	DWORD 		bfOffBits;  //bitmap data offset
//} BITMAPFILEHEADER;
//
//typedef struct tagBITMAPINFOHEADER
//{
//	DWORD biSize;    // info header size
//	LONG biWidth;
//	LONG biHeight;
//	WORD biPlanes;
//	WORD biBitCount;	//1, 4, 8, 16, 24 or 32
//	DWORD biCompression; //0:BI_RGB, 1:BI_RLE8, 2: BI_RLE4, 3:BI_BITFIELDS, 4:BI_JPEG, 5:BI_PNG
//	DWORD biSizeImage;   // if BI_RGB, can set be 0
//	LONG biXPelsPerMeter;
//	LONG biYPelsPerMeter;
//	DWORD biClrUsed;
//	DWORD biClrImportant;
//} BITMAPINFOHEADER;
//
//FRESULT saveScreenBigmap(FIL *fp);
//uint8_t saveScreen(TCHAR* path);
//FRESULT bitmapHeader(FIL* fp, BITMAPFILEHEADER *fh, BITMAPINFOHEADER *ih);



 
