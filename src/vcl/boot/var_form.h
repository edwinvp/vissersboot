//---------------------------------------------------------------------------

#ifndef var_formH
#define var_formH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include <memory>
//---------------------------------------------------------------------------
class TVarForm : public TForm
{
__published:	// IDE-managed Components
	TMemo *Memo1;
private:	// User declarations
	std::unique_ptr<TStringList> Lines;
public:		// User declarations
	__fastcall TVarForm(TComponent* Owner);

	void __fastcall ClearLines();
	void __fastcall AddLine(UnicodeString s);
	void __fastcall Update();

};
//---------------------------------------------------------------------------
extern PACKAGE TVarForm *VarForm;
//---------------------------------------------------------------------------
#endif
