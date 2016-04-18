//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "mainform.h"
#include "fakeio.h"
#include "var_form.h"
#include "vboot.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TMainFrm *MainFrm;
//---------------------------------------------------------------------------
#define pi 3.1415926535897932384626433832795f
//---------------------------------------------------------------------------
__fastcall TMainFrm::TMainFrm(TComponent* Owner)
	: TForm(Owner)
{
	//da.SetZoomFactor(8400000);
	da.SetZoomFactor(10000);

	vessel.heading = 40.0f;
	vessel.position.lat = 51.9364818f;
	vessel.position.lon = 4.5162849f;
	vessel.speed = 0;

	AddLocations();

	da.SetCenterLoc(start.loc);

	VarForm = new TVarForm(Application);
	VarForm->Show();

	UnicodeString fnPuttyLog=L"C:\\Users\\Z\\Desktop\\putty.log";

	putty_log_file.reset(new TFileStream(fnPuttyLog,fmOpenRead | fmShareDenyNone));
	putty_log_file->Position = putty_log_file->Size;

	main_init();

}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::AddRefLocations()
{
	TDrawPoint zero,utrecht,amsterdam,rotterdam;

	// zero loc
	zero.clr = clBlack;
	zero.loc.lat = 0;
	zero.loc.lon = 0;
	da.AddDrawPoint(zero);
	// Utrecht
	utrecht.clr = clYellow;
	utrecht.loc.lat = 52.0906949f;
	utrecht.loc.lon = 5.1220616f;
	da.AddDrawPoint(utrecht);
	// Amsterdam
	amsterdam.clr = clRed;
	amsterdam.loc.lat = 52.3788516f;
	amsterdam.loc.lon = 4.9004368f;
	da.AddDrawPoint(amsterdam);
	// Rotterdam
	rotterdam.clr = clGreen;
	rotterdam.loc.lat = 51.9290846f;
	rotterdam.loc.lon = 4.4931016f;
	da.AddDrawPoint(rotterdam);

	// Kralingse plas
	TDrawPoint kp1, kp2, kp3,kp4;
	// kp1  kp2
	// kp3  kp4
	kp1.clr = clBlue;
	kp2.clr = clBlue;
	kp3.clr = clBlue;
	kp4.clr = clBlue;
	kp1.loc.lat = 51.9413472f;
	kp1.loc.lon = 4.513745f;
	kp2.loc.lat = 51.9380235f;
	kp2.loc.lon = 4.5250358f;
	kp3.loc.lat = 51.9340276f;
	kp3.loc.lon = 4.5056552f;
	kp4.loc.lat = 51.9281928f;
	kp4.loc.lon = 4.5183095f;
	da.AddDrawPoint(kp1);
	da.AddDrawPoint(kp2);
	da.AddDrawPoint(kp3);
	da.AddDrawPoint(kp4);
}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::AddLocations()
{
	AddRefLocations();

	// startpunt navigatie: ergens midden in de Kralingse plas
	start.clr = clWhite;
	start.loc.lat = 51.9364818f;
	start.loc.lon = 4.5162849f;
	da.AddDrawPoint(start);
	// eindpunt/finish navigatie: wat verder weg (voor de tuin)
	finish.clr = clWhite;
	finish.loc.lat = 51.932278f;
	finish.loc.lon = 4.521163f;
	da.AddDrawPoint(finish);

	gp_start.lat = start.loc.lat;
	gp_start.lon = start.loc.lon;

	gp_mem_1.lat = finish.loc.lat;
	gp_mem_1.lon = finish.loc.lon;

	gp_mem_2.lat = start.loc.lat;
	gp_mem_2.lon = start.loc.lon;

	gp_finish.lat = finish.loc.lat;
	gp_finish.lon = finish.loc.lon;
}
//---------------------------------------------------------------------------
float __fastcall TMainFrm::Pwm2MotorFact(int dc)
{
	if (!dc)
		return 0.0f;
	else
		return (dc-3000)/1000.0f;
}
//---------------------------------------------------------------------------
UnicodeString MainStateToText(TMainState s)
{
	switch (s) {
	case msManualMode: return L"msManualMode";
	case msAutoMode: return L"msAutoMode";
	case msCountJoyGoto: return L"msCountJoyGoto";
	case msConfirmGotoPosX: return L"msConfirmGotoPosX";
	case msCountJoyStore: return L"msCountJoyStore";
	case msConfirmStorePosX: return L"msConfirmStorePosX";
	case msCountJoyGotoRetn: return L"msCountJoyGotoRetn";
	case msCountJoyStoreRetn: return L"msCountJoyStoreRetn";
	case msClear1: return L"msClear1";
	case msClear2: return L"msClear2";

	case msCmdErrorMan: return L"msCmdErrorMan";
	case msCmdErrorAuto: return L"msCmdErrorAuto";

	default:
		return L"???";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::Timer1Timer(TObject *Sender)
{
	global_ms_timer += Timer1->Interval;

	if (CbRemoteCtrlOn->Checked) {
		pd5_pulse_duration=SbMotorV->Position;
		pd6_pulse_duration=SbMotorH->Position;
		pd3_pulse_duration=4000 - (SbPos->Position - 2000);
		pb3_pulse_duration=SbManAuto->Position;
	} else {
		// If the remote is off, no servo waveform is
		// generated by the receiver. So zeroes.
		pd6_pulse_duration=0;
		pd5_pulse_duration=0;
		pd3_pulse_duration=0;
		pb3_pulse_duration=0;
	}

	// run atmel program
	SendVesselPosToAtmel();
	process();

	if (prog_op.Length()) {
		if (ListBox1->Items->Count > 5)
			ListBox1->Clear();

		ListBox1->Items->Add(prog_op);
		prog_op="";
	}

	if (PORTB & _BV(PORTB5))
		ShLed->Brush->Color=clRed;
	else
		ShLed->Brush->Color=clBlack;

	vessel.bearing_sp = bearing_sp;

	float avr_motor_l(0),avr_motor_r(0);

	avr_motor_l = Pwm2MotorFact(OCR1A);
	avr_motor_r = Pwm2MotorFact(OCR1B);

	vessel.motor_left = avr_motor_l -(SbMotorL->Position / 100.0);
	vessel.motor_right = avr_motor_r -(SbMotorR->Position / 100.0);

	vessel_path.push_back(vessel.position);

	vessel.CalcSpeedAndHeading();
	vessel.Move(Timer1->Interval);
	//vessel.heading += 0.2;

	da.SetScreenDims(PaintBox1->Width,PaintBox1->Height);

	da.RenderTo(PaintBox1->Canvas,vessel,vessel_path);


	VarForm->ClearLines();

	VarForm->AddLine(L"vessel.motor_left = " + FormatFloat(L"0.00",vessel.motor_left));
	VarForm->AddLine(L"vessel.motor_right = " + FormatFloat(L"0.00",vessel.motor_right));
	VarForm->AddLine(L"---");
	VarForm->AddLine(L"gp_mem_1 = " + gp_mem_1.ToString());
	VarForm->AddLine(L"gp_mem_2 = " + gp_mem_2.ToString());
	VarForm->AddLine(L"gp_mem_3 = " + gp_mem_3.ToString());
	VarForm->AddLine(L"---");
	VarForm->AddLine(L"gp_current = " + gp_current.ToString());
	VarForm->AddLine(L"distance_m = "+ FloatToStr(distance_m));
	VarForm->AddLine(L"arrived = "+ IntToStr((int)arrived));

	VarForm->AddLine(L"---");
	VarForm->AddLine(L"main_state = " + MainStateToText(main_state));
	VarForm->AddLine(L"state_time = " + IntToStr((int)state_time));
	VarForm->AddLine(L"joy_pulses = " + IntToStr(joy_pulses));

	VarForm->AddLine(L"---");
	VarForm->AddLine(L"pd3 pd = " + FloatToStr(pd3_pulse_duration));
	VarForm->AddLine(L"pb3 pd = " + FloatToStr(pb3_pulse_duration));
	VarForm->AddLine(L"pd5 pd = " + FloatToStr(pd5_pulse_duration));
	VarForm->AddLine(L"pd6 pd = " + FloatToStr(pd6_pulse_duration));

	VarForm->AddLine(L"---");
	VarForm->AddLine(L"p_add = " + FloatToStr(p_add));
	VarForm->AddLine(L"i_add = " + FloatToStr(i_add));
	VarForm->AddLine(L"d_add = " + FloatToStr(d_add));


	VarForm->Update();


}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::SendVesselPosToAtmel()
{
	TGpsLoc loc = vessel.position;
//	loc.lat = -89.0;
//	loc.lon = -179.0;

	bool bValidity = CbValidGps->Checked;

	// Convert current location to a GPS string parseable by
	// the TinyGPS library.
	UnicodeString sRMC = loc.GetGPRMC(vessel.heading,bValidity);

	// Send as fake UART message
	for (int i(1);i<=sRMC.Length();++i) {
		Fake_UART_ISR(sRMC[i]);
		process();
	}
}
//---------------------------------------------------------------------------

void __fastcall TMainFrm::OnZoomFactKeyDown(TObject *Sender, WORD &Key, TShiftState Shift)

{
	if (Key==VK_RETURN) {
		da.SetZoomFactor(EdZoomFactor->Text.ToDouble());
	}
}
//---------------------------------------------------------------------------

void __fastcall TMainFrm::BtnZeroClick(TObject *Sender)
{
	SbMotorL->Position=0;
	SbMotorR->Position=0;
}
//---------------------------------------------------------------------------

void __fastcall TMainFrm::BtnAutoModeClick(TObject *Sender)
{
	main_state = msAutoMode;
}
//---------------------------------------------------------------------------


void __fastcall TMainFrm::Timer2Timer(TObject *Sender)
{
	if (putty_log_file->Position < putty_log_file->Size) {

		int avail = putty_log_file->Size - putty_log_file->Position;

		int to_read = avail;
		if (avail>80) {
			avail = 80;
		}


		std::vector<char> v(to_read);
		putty_log_file->ReadBuffer(&v[0],to_read);

		for (unsigned int i(0);i<v.size();i++) {
			if (v[i] == 0x0d || v[i] == 0x0a) {
				AnsiString u = AnsiString(s.c_str()).Trim();
				if (u.Length())
					NewXYZStr(u);
				s.clear();
			} else
				s += v[i];
		}

	}
}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::C2Scr(Graphics::TBitmap * bmp, int & sx, int & sy, float x, float y)
{
	int cx(bmp->Width/2),cy(bmp->Height/2);

	double rel_x = (x / 1500.0) * cx;
	double rel_y = -(y / 1500.0) * cy;
	sx = cx + rel_x;
	sy = cy + rel_y;
}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::NewXYZStr(AnsiString s)
{
	int x(0),y(0),z(0);
	int nf = sscanf(s.c_str(),"x=%d, y=%d, z=%d",&x,&y,&z);


	ext_compass_x = x;
	ext_compass_y = y;
	ext_compass_z = z;


	if (nf == 3) {
		TCompassTriple t;
		t.x = x;
		t.y = y;
		t.z = z;
		cvalues.push_back(t);
	}

	std::auto_ptr<Graphics::TBitmap> bmp(new Graphics::TBitmap());
	bmp->Width = PaintBox2->Width;
	bmp->Height = PaintBox2->Height;

	std::deque<TCompassTriple>::const_iterator it;
	for (it=cvalues.begin();it!=cvalues.end();++it) {
		int px(0),py(0);

		const TCompassTriple & t = *it;
		C2Scr(bmp.get(),px,py,t.x,t.z);

		bmp->Canvas->Ellipse(px-4,py-4,px+4,py+4);
	}

	int sx(0),sy(0);
	C2Scr(bmp.get(),sx,sy,compass_min_x.fin,compass_min_z.fin);
	bmp->Canvas->MoveTo(sx,sy);
	C2Scr(bmp.get(),sx,sy,compass_min_x.fin,compass_max_z.fin);
	bmp->Canvas->LineTo(sx,sy);
	C2Scr(bmp.get(),sx,sy,compass_max_x.fin,compass_max_z.fin);
	bmp->Canvas->LineTo(sx,sy);
	C2Scr(bmp.get(),sx,sy,compass_max_x.fin,compass_min_z.fin);
	bmp->Canvas->LineTo(sx,sy);
	C2Scr(bmp.get(),sx,sy,compass_min_x.fin,compass_min_z.fin);
	bmp->Canvas->LineTo(sx,sy);

	TCompassTriple centered;
	centered.x=0;
	centered.y=0;
	centered.z=0;

	if (!cvalues.empty()) {
		const TCompassTriple & t = *cvalues.rbegin();

		float w = compass_max_x.fin - compass_min_x.fin;
		float h = compass_max_z.fin - compass_min_z.fin;

		if (w>=0 && h>=0) {
			centered.x = (t.x - compass_min_x.fin - (w/2.0));
			centered.z = -(t.z - compass_min_z.fin - (h/2.0));
		}
	}

	bmp->Canvas->Pen->Color = clBlue;

	int sr(bmp->Width);
	if (sr > bmp->Height)
		sr = bmp->Height;

	int cx(bmp->Width/2);
	int cy(bmp->Height/2);
	float rIdeal = 0.4 * sr;

	bmp->Canvas->Brush->Style = bsClear;
	bmp->Canvas->Ellipse(cx - rIdeal,cy - rIdeal,cx + rIdeal,cy + rIdeal);

	bmp->Canvas->Brush->Style = bsSolid;
	bmp->Canvas->Brush->Color = clBlue;

	float w = compass_max_x.fin - compass_min_x.fin;
	float h = compass_max_z.fin - compass_min_z.fin;


	// True heading, 0=N, 270=W etc...
	float d = compass_course;

	bmp->Canvas->Pen->Color = clRed;

	sx = cx + cos((d-90)/360.0*2.0*pi) * rIdeal;
	sy = cy + sin((d-90)/360.0*2.0*pi) * rIdeal;

	bmp->Canvas->MoveTo(cx,cy);
	bmp->Canvas->LineTo(sx,sy);

	PaintBox2->Canvas->Draw(0,0,bmp.get());

}
