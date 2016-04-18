//---------------------------------------------------------------------------

#ifndef mainformH
#define mainformH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include <Vcl.ExtCtrls.hpp>
//---------------------------------------------------------------------------
#include "drawing_area.h"
#include <Vcl.ComCtrls.hpp>
#include <deque>
#include "vboot.h"
//---------------------------------------------------------------------------
class TMainFrm : public TForm
{
__published:	// IDE-managed Components
	TTimer *Timer1;
	TSplitter *Splitter1;
	TPanel *Panel1;
	TListBox *ListBox1;
	TGroupBox *GroupBox1;
	TShape *ShLed;
	TCheckBox *CbRemoteCtrlOn;
	TGroupBox *GbRemoteCtrl;
	TScrollBar *SbPos;
	TScrollBar *SbMotorV;
	TScrollBar *SbManAuto;
	TScrollBar *SbMotorH;
	TLabeledEdit *EdZoomFactor;
	TLabel *Label1;
	TScrollBar *SbMotorL;
	TScrollBar *SbMotorR;
	TPanel *PnlLegenda;
	TShape *Shape1;
	TLabel *Label2;
	TLabel *Label3;
	TShape *Shape2;
	TShape *Shape3;
	TLabel *Label4;
	TLabel *Label5;
	TLabel *Label6;
	TLabel *Label7;
	TButton *BtnZero;
	TCheckBox *CbValidGps;
	TButton *BtnAutoMode;
	TPageControl *PageControl1;
	TTabSheet *TsCourseOverview;
	TTabSheet *TsMagneto;
	TPaintBox *PaintBox1;
	TPaintBox *PaintBox2;
	TTimer *Timer2;
	void __fastcall Timer1Timer(TObject *Sender);
	void __fastcall OnZoomFactKeyDown(TObject *Sender, WORD &Key, TShiftState Shift);
	void __fastcall BtnZeroClick(TObject *Sender);
	void __fastcall BtnAutoModeClick(TObject *Sender);
	void __fastcall Timer2Timer(TObject *Sender);

private:	// User declarations
	TDrawingArea da;
	TVessel vessel;
	TDrawPoint start,finish;
	std::vector<TGpsLoc> vessel_path;
	std::auto_ptr<TFileStream> putty_log_file;

	void __fastcall SendVesselPosToAtmel();
	void __fastcall AddLocations();
	void __fastcall AddRefLocations();
	float __fastcall Pwm2MotorFact(int dc);

	std::string s;
	void __fastcall NewXYZStr(AnsiString s);

	std::deque<TCompassTriple> cvalues;

	void __fastcall C2Scr(Graphics::TBitmap * bmp, int & sx, int & sy, float x, float y);

public:		// User declarations
	__fastcall TMainFrm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TMainFrm *MainFrm;
//---------------------------------------------------------------------------
#endif
