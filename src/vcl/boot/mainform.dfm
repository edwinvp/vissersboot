object MainFrm: TMainFrm
  Left = 0
  Top = 0
  Caption = 'Main simulation form'
  ClientHeight = 494
  ClientWidth = 725
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  Position = poOwnerFormCenter
  PixelsPerInch = 96
  TextHeight = 13
  object PaintBox1: TPaintBox
    Left = 0
    Top = 0
    Width = 423
    Height = 402
    Align = alClient
    ExplicitLeft = 16
    ExplicitTop = -6
  end
  object Splitter1: TSplitter
    Left = 423
    Top = 0
    Width = 16
    Height = 402
    Align = alRight
    Beveled = True
    ExplicitLeft = 417
    ExplicitTop = 8
  end
  object Panel1: TPanel
    Left = 439
    Top = 0
    Width = 286
    Height = 402
    Align = alRight
    TabOrder = 0
    ExplicitLeft = 289
    ExplicitWidth = 311
    ExplicitHeight = 312
    object GroupBox1: TGroupBox
      Left = 6
      Top = 207
      Width = 59
      Height = 66
      Caption = 'LED'
      TabOrder = 0
      object ShLed: TShape
        Left = 16
        Top = 24
        Width = 25
        Height = 25
      end
    end
    object CbRemoteCtrlOn: TCheckBox
      Left = 6
      Top = 9
      Width = 113
      Height = 25
      Caption = 'Remote control ON'
      Checked = True
      State = cbChecked
      TabOrder = 1
    end
    object GbRemoteCtrl: TGroupBox
      Left = 6
      Top = 40
      Width = 258
      Height = 161
      Caption = 'Remote'
      TabOrder = 2
      object SbPos: TScrollBar
        Left = 64
        Top = 47
        Width = 17
        Height = 97
        Kind = sbVertical
        Max = 4000
        Min = 2000
        PageSize = 0
        Position = 3000
        TabOrder = 0
      end
      object SbMotorV: TScrollBar
        Left = 176
        Top = 47
        Width = 17
        Height = 97
        Kind = sbVertical
        Max = 4000
        Min = 2000
        PageSize = 0
        Position = 3000
        TabOrder = 1
      end
      object SbManAuto: TScrollBar
        Left = 22
        Top = 24
        Width = 97
        Height = 17
        Max = 4000
        Min = 2000
        PageSize = 0
        Position = 3000
        TabOrder = 2
      end
      object SbMotorH: TScrollBar
        Left = 134
        Top = 24
        Width = 97
        Height = 17
        Max = 4000
        Min = 2000
        PageSize = 0
        Position = 3000
        TabOrder = 3
      end
    end
  end
  object ListBox1: TListBox
    Left = 0
    Top = 402
    Width = 725
    Height = 92
    Align = alBottom
    ItemHeight = 13
    TabOrder = 1
    ExplicitTop = 312
    ExplicitWidth = 600
  end
  object Timer1: TTimer
    Interval = 100
    OnTimer = Timer1Timer
    Left = 128
    Top = 128
  end
end
