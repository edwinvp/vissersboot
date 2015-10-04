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
    Width = 368
    Height = 402
    Align = alClient
    ExplicitWidth = 344
  end
  object Splitter1: TSplitter
    Left = 368
    Top = 0
    Width = 16
    Height = 402
    Align = alRight
    Beveled = True
    ExplicitLeft = 417
    ExplicitTop = 8
  end
  object Panel1: TPanel
    Left = 384
    Top = 0
    Width = 341
    Height = 402
    Align = alRight
    TabOrder = 0
    object Label1: TLabel
      Left = 96
      Top = 280
      Width = 88
      Height = 13
      Caption = 'Full power: 5 km/u'
    end
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
    object Button1: TButton
      Left = 6
      Top = 338
      Width = 89
      Height = 41
      Caption = 'Button1'
      TabOrder = 3
      OnClick = Button1Click
    end
    object Edit1: TEdit
      Left = 6
      Top = 306
      Width = 419
      Height = 21
      TabOrder = 4
    end
    object EdZoomFactor: TLabeledEdit
      Left = 112
      Top = 235
      Width = 121
      Height = 21
      EditLabel.Width = 69
      EditLabel.Height = 13
      EditLabel.Caption = 'EdZoomFactor'
      TabOrder = 5
      Text = '100'
      OnKeyDown = OnZoomFactKeyDown
    end
    object SbMotorL: TScrollBar
      Left = 270
      Top = 79
      Width = 25
      Height = 194
      Kind = sbVertical
      Min = -100
      PageSize = 0
      TabOrder = 6
    end
    object SbMotorR: TScrollBar
      Left = 294
      Top = 79
      Width = 25
      Height = 194
      Kind = sbVertical
      Min = -100
      PageSize = 0
      TabOrder = 7
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
  end
  object Timer1: TTimer
    Interval = 100
    OnTimer = Timer1Timer
    Left = 128
    Top = 128
  end
end
