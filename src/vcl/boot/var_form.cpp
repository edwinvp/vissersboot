//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "var_form.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TVarForm *VarForm;
//---------------------------------------------------------------------------
__fastcall TVarForm::TVarForm(TComponent* Owner)
	: TForm(Owner)
{
	Lines.reset(new TStringList());
}
//---------------------------------------------------------------------------
void __fastcall TVarForm::ClearLines()
{
	Lines->Clear();
}
//---------------------------------------------------------------------------
void __fastcall TVarForm::AddLine(UnicodeString s)
{
	Lines->Add(s);
}
//---------------------------------------------------------------------------
void __fastcall TVarForm::Update()
{
	Memo1->Lines->Assign(Lines.get());
}
//---------------------------------------------------------------------------
