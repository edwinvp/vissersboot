//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "mainform.h"
#include "fakeio.h"
#include "var_form.h"
#include "compass_calibrate.h"
#include "waypoints.h"
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
	//da.SetZoomFactor(10000); // kral. plas
	da.SetZoomFactor(80000); // lage bergse plas
	da.SetZoomFactor(300000); // view of boat

	vessel.heading = 40.0f;

#if 0
	// kralingse plas
	vessel.position.lat = 51.9364818f;
	vessel.position.lon = 4.5162849f;
	vessel.speed = 0;
#endif

#if 1
	// lage bergse bos, ergens op het meertje
	vessel.position.lat = 51.966982;
	vessel.position.lon = 4.515443;
	vessel.speed = 0;
#endif

	AddLocations();

	da.SetCenterLoc(start.loc);

	VarForm = new TVarForm(Application);
	VarForm->Show();

	UnicodeString fnPuttyLog=L"C:\\Users\\Z\\Desktop\\putty.log";

	try {
		putty_log_file.reset(new TFileStream(fnPuttyLog,fmOpenRead | fmShareDenyNone));
		putty_log_file->Position = putty_log_file->Size;
	} catch (Exception & e) {
		putty_log_file.reset(0);
	}


	main_init();

}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::AddRefpoint(double lat, double lon, TColor clr)
{
	TDrawPoint rp(lat,lon);
	rp.clr = clr;
	da.AddDrawPoint(rp);
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
	// kp1  kp2
	// kp3  kp4
	AddRefpoint(51.9413472f,4.513745f,clBlue);
	AddRefpoint(51.9380235f,4.5250358f,clBlue);
	AddRefpoint(51.9340276f,4.5056552f,clBlue);
	AddRefpoint(51.9281928f,4.5183095f,clBlue);

	// Lage Bergse Bos
	AddRefpoint(51.9413472f,4.513745f,clBlue);
	AddRefpoint(51.966493, 4.514893,clBlue);
	AddRefpoint(51.966907, 4.514823,clBlue);
	AddRefpoint(51.967109, 4.514874,clBlue);
	AddRefpoint(51.967203, 4.514955,clBlue);
	AddRefpoint(51.967335, 4.515105,clBlue);
	AddRefpoint(51.967548, 4.515274,clBlue);
	AddRefpoint(51.967626, 4.515247,clBlue);
	AddRefpoint(51.967738, 4.515115,clBlue);
	AddRefpoint(51.966486, 4.515470,clBlue);
	AddRefpoint(51.966388, 4.515727,clBlue);
	AddRefpoint(51.966415, 4.515864,clBlue);
	AddRefpoint(51.966492, 4.514898,clBlue);
	AddRefpoint(51.966750, 4.514595,clBlue);
	AddRefpoint(51.966521, 4.515032,clBlue);
	AddRefpoint(51.966570, 4.514753,clBlue);
	AddRefpoint(51.966638, 4.514665,clBlue);


}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::AddLocations()
{
	AddRefLocations();

#if 0
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
#endif

#if 1
	// startpunt navigatie: lage bergse bos, parkeerplaats/speeltuin
	start.clr = clWhite;
	start.loc.lat = 51.966982;
	start.loc.lon = 4.515443;
	da.AddDrawPoint(start);

	// eindpunt/finish navigatie: wat verder weg (voor de tuin)
	finish.clr = clWhite;
	finish.loc.lat = 51.966572;
	finish.loc.lon = 4.51526;
	da.AddDrawPoint(finish);
#endif


	waypoints.gp_start.lat = start.loc.lat;
	waypoints.gp_start.lon = start.loc.lon;

	waypoints.gp_mem_1.lat = finish.loc.lat;
	waypoints.gp_mem_1.lon = finish.loc.lon;

	waypoints.gp_mem_2.lat = start.loc.lat;
	waypoints.gp_mem_2.lon = start.loc.lon;

	waypoints.gp_finish.lat = finish.loc.lat;
	waypoints.gp_finish.lon = finish.loc.lon;

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
	case msAutoModeNormal: return L"msAutoModeNormal";
	case msReverseThrust: return L"msReverseThrust";
	case msAutoModeCourse: return L"msAutoModeCourse";
	case msCountJoyGoto: return L"msCountJoyGoto";
	case msConfirmGotoPosX: return L"msConfirmGotoPosX";
	case msCountJoyStore: return L"msCountJoyStore";
	case msConfirmStorePosX: return L"msConfirmStorePosX";
	case msCountJoyGotoRetn: return L"msCountJoyGotoRetn";
	case msCountJoyStoreRetn: return L"msCountJoyStoreRetn";
	case msClear1: return L"msClear1";
	case msClear2: return L"msClear2";
	case msConfirmClear: return L"msConfirmClear";

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
		pb4_pulse_duration=SbManAuto->Position;
	} else {
		// If the remote is off, no servo waveform is
		// generated by the receiver. So zeroes.
		pd6_pulse_duration=0;
		pd5_pulse_duration=0;
		pd3_pulse_duration=0;
		pb4_pulse_duration=0;
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

	if (PORTD & _BV(PORTD2))
		ShHeadLight->Brush->Color=clRed;
	else
		ShHeadLight->Brush->Color=clBlack;

	vessel.compass_course = cc.compass_course;
	vessel.bearing_sp = steering.bearing_sp;

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
	VarForm->AddLine(L"gp_mem_1 = " + waypoints.gp_mem_1.ToString());
	VarForm->AddLine(L"gp_mem_2 = " + waypoints.gp_mem_2.ToString());
	VarForm->AddLine(L"gp_mem_3 = " + waypoints.gp_mem_3.ToString());
	VarForm->AddLine(L"---");
	VarForm->AddLine(L"gp_current = " + waypoints.gp_current.ToString());
	VarForm->AddLine(L"distance_m = "+ FloatToStr(distance_m));
	VarForm->AddLine(L"arrived = "+ IntToStr((int)steering.arrived));

	VarForm->AddLine(L"---");
	VarForm->AddLine(L"main_state = " + MainStateToText(stm.Step()));
	VarForm->AddLine(L"state_time = " + IntToStr((int)stm.TimeInStep()));
	VarForm->AddLine(L"joy_pulses = " + IntToStr(stm.joy_pulses));

	VarForm->AddLine(L"---");
	VarForm->AddLine(L"pd3 pd = " + FloatToStr(pd3_pulse_duration));
	VarForm->AddLine(L"pb4 pd = " + FloatToStr(pb4_pulse_duration));
	VarForm->AddLine(L"pd5 pd = " + FloatToStr(pd5_pulse_duration));
	VarForm->AddLine(L"pd6 pd = " + FloatToStr(pd6_pulse_duration));

	VarForm->AddLine(L"---");
	VarForm->AddLine(L"p_add = " + FloatToStr(steering.p_add));
	VarForm->AddLine(L"i_add = " + FloatToStr(steering.i_add));
	VarForm->AddLine(L"d_add = " + FloatToStr(steering.d_add));
	VarForm->AddLine(L"cv(clipped) = " + FormatFloat("0.00",steering.cv_clipped));

	TbMotorL->Position = steering.get_motor_L_perc() * 100.0;
	TbMotorR->Position = steering.get_motor_R_perc() * 100.0;


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
		gps_buffer.push(sRMC[i]);
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
	stm.ForceStep(msAutoModeCourse);
}
//---------------------------------------------------------------------------


void __fastcall TMainFrm::Timer2Timer(TObject *Sender)
{
	if (CbUseRealCompass->Checked) {
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
	} else {

#if 0
		// Convert trackbar to 'fake' compass reading
		// Use scrollbar
		float degs = CompassFollowsTrackbar();
#else
		// Use virtual vessel:
		float degs = CompassFollowsVessel() + (rand()%20-5)/10.0f;
#endif

		double phi = degs/360.0*2.0*pi;

		double rx = (cc.mm_x.fin_max - cc.mm_x.fin_min)/2;
		double rz = (cc.mm_z.fin_max - cc.mm_z.fin_min)/2;
		double cent_x = cc.mm_x.fin_min + rx;
		double cent_z = cc.mm_z.fin_min + rz;

		int x = cent_x - sin(phi)*rx;
		int y = 0;
		int z = cent_z - cos(phi)*rz;

		if (!CbFreezeCompass->Checked)
			ThreeNewCompassValues(x,y,z);
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
void __fastcall TMainFrm::ThreeNewCompassValues(int x, int y, int z)
{
	TCompassTriple t;
	t.x.value = x;
	t.y.value = y;
	t.z.value = z;
	cvalues.push_back(t);

	ext_compass_x = x;
	ext_compass_y = y;
	ext_compass_z = z;


	std::auto_ptr<Graphics::TBitmap> bmp(new Graphics::TBitmap());
	bmp->Width = PaintBox2->Width;
	bmp->Height = PaintBox2->Height;

	std::deque<TCompassTriple>::const_iterator it;
	for (it=cvalues.begin();it!=cvalues.end();++it) {
		int px(0),py(0);

		const TCompassTriple & t = *it;
		C2Scr(bmp.get(),px,py,t.x.value,t.z.value);

		bmp->Canvas->Ellipse(px-4,py-4,px+4,py+4);
	}

	// Show entire (normal) sensor min/max range as rectangle
	int x1,y1,x2,y2;
	C2Scr(bmp.get(),x1,y1,cc.mm_x.fin_min,cc.mm_z.fin_min);
	C2Scr(bmp.get(),x2,y2,cc.mm_x.fin_max,cc.mm_z.fin_max);
	bmp->Canvas->Brush->Style = bsClear;
	bmp->Canvas->Rectangle(x1,y1,x2,y2);
	bmp->Canvas->Brush->Style = bsSolid;


	TCompassTriple centered;
	centered.x.value=0;
	centered.y.value=0;
	centered.z.value=0;

	if (!cvalues.empty()) {
		const TCompassTriple & t = *cvalues.rbegin();

		float w = cc.mm_x.fin_max - cc.mm_x.fin_min;
		float h = cc.mm_z.fin_max - cc.mm_z.fin_min;

		if (w>=0 && h>=0) {
			centered.x.value = (t.x.value - cc.mm_x.fin_min - (w/2.0));
			centered.z.value = -(t.z.value - cc.mm_z.fin_min - (h/2.0));
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

	float w = cc.mm_x.fin_max - cc.mm_x.fin_min;
	float h = cc.mm_z.fin_max - cc.mm_z.fin_min;


	// True heading, 0=N, 270=W etc...
	float d = cc.compass_course;

	bmp->Canvas->Pen->Color = clRed;

	int sx = cx + cos((d-90)/360.0*2.0*pi) * rIdeal;
	int sy = cy + sin((d-90)/360.0*2.0*pi) * rIdeal;

	bmp->Canvas->MoveTo(cx,cy);
	bmp->Canvas->LineTo(sx,sy);

	PaintBox2->Canvas->Draw(0,0,bmp.get());
}
//---------------------------------------------------------------------------
void __fastcall TMainFrm::NewXYZStr(AnsiString s)
{
	int x(0),y(0),z(0);
	int nf = sscanf(s.c_str(),"x=%d, y=%d, z=%d",&x,&y,&z);

	if (nf == 3) {
		ThreeNewCompassValues(x,y,z);
	}


}
void __fastcall TMainFrm::OnInputKeyPress(TObject *Sender, System::WideChar &Key)

{
	char c(0);

	if ((Key>='a' && Key<='z') ||
		(Key>='0' && Key<='9') ||
		(Key>='-') ||
		(Key>='+') ||
		(Key>='\r') ||
		(Key>='\n'))
		c=Key;

	Fake_UART_ISR(c);
}
//---------------------------------------------------------------------------
float __fastcall TMainFrm::CompassFollowsTrackbar()
{
	// Convert trackbar to 'fake' compass reading
	// Use scrollbar
	static float degs(0);

	float delta(0.0);
	delta =TbCompass->Position / 100.0;

	if (fabs(delta) > 5.0)
		degs += delta;

	if (degs >= 360)
		degs-=360;
	if (degs < 0)
		degs+=360;

	// Act as if compass calibration yielded these values
	cc.mm_x.fin_min = -1000;
	cc.mm_z.fin_min = -1000;
	cc.mm_x.fin_max = 1000;
	cc.mm_z.fin_max = 1000;

	return degs;
}
//---------------------------------------------------------------------------
float __fastcall TMainFrm::CompassFollowsVessel()
{
	// Act as if compass calibration yielded these values
	cc.mm_x.fin_min = -1000;
	cc.mm_z.fin_min = -1000;
	cc.mm_x.fin_max = 1000;
	cc.mm_z.fin_max = 1000;

	return vessel.heading;
}

//---------------------------------------------------------------------------


