object MainFrm: TMainFrm
  Left = 0
  Top = 0
  Caption = 'Main simulation form'
  ClientHeight = 635
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
  object Splitter1: TSplitter
    Left = 368
    Top = 0
    Width = 16
    Height = 484
    Align = alRight
    Beveled = True
    ExplicitLeft = 417
    ExplicitTop = 8
    ExplicitHeight = 402
  end
  object Panel1: TPanel
    Left = 384
    Top = 0
    Width = 341
    Height = 484
    Align = alRight
    TabOrder = 0
    object Label1: TLabel
      Left = 96
      Top = 280
      Width = 88
      Height = 13
      Caption = 'Full power: 5 km/u'
    end
    object Label5: TLabel
      Left = 280
      Top = 40
      Width = 33
      Height = 13
      Caption = 'Motors'
    end
    object Label6: TLabel
      Left = 272
      Top = 56
      Width = 13
      Height = 13
      Caption = 'ML'
    end
    object Label7: TLabel
      Left = 300
      Top = 56
      Width = 15
      Height = 13
      Caption = 'MR'
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
    object EdZoomFactor: TLabeledEdit
      Left = 136
      Top = 236
      Width = 121
      Height = 21
      EditLabel.Width = 69
      EditLabel.Height = 13
      EditLabel.Caption = 'EdZoomFactor'
      TabOrder = 3
      Text = '300000'
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
      TabOrder = 4
    end
    object SbMotorR: TScrollBar
      Left = 294
      Top = 79
      Width = 25
      Height = 194
      Kind = sbVertical
      Min = -100
      PageSize = 0
      TabOrder = 5
    end
    object PnlLegenda: TPanel
      Left = 6
      Top = 299
      Width = 209
      Height = 73
      TabOrder = 6
      object Shape1: TShape
        Left = 16
        Top = 16
        Width = 25
        Height = 9
        Brush.Color = clSilver
      end
      object Label2: TLabel
        Left = 47
        Top = 12
        Width = 75
        Height = 13
        Caption = 'Vessel points to'
      end
      object Label3: TLabel
        Left = 47
        Top = 32
        Width = 124
        Height = 13
        Caption = 'Initial bearing (Haversine)'
      end
      object Shape2: TShape
        Left = 16
        Top = 36
        Width = 25
        Height = 9
        Brush.Color = clBlue
      end
      object Shape3: TShape
        Left = 16
        Top = 55
        Width = 25
        Height = 9
        Brush.Color = clMaroon
      end
      object Label4: TLabel
        Left = 47
        Top = 51
        Width = 113
        Height = 13
        Caption = 'GPS Course Made Good'
      end
    end
    object BtnZero: TButton
      Left = 272
      Top = 279
      Width = 57
      Height = 26
      Caption = 'Zero'
      TabOrder = 7
      OnClick = BtnZeroClick
    end
    object CbValidGps: TCheckBox
      Left = 6
      Top = 378
      Width = 225
      Height = 17
      Caption = 'Send valid GPS position to AVR program'
      Checked = True
      State = cbChecked
      TabOrder = 8
    end
    object BtnAutoMode: TButton
      Left = 221
      Top = 341
      Width = 105
      Height = 33
      Caption = 'Force Auto Mode'
      TabOrder = 9
      OnClick = BtnAutoModeClick
    end
    object EdInput: TLabeledEdit
      Left = 6
      Top = 407
      Width = 153
      Height = 21
      EditLabel.Width = 38
      EditLabel.Height = 13
      EditLabel.Caption = 'EdInput'
      TabOrder = 10
      OnKeyPress = OnInputKeyPress
    end
    object CbUseRealCompass: TCheckBox
      Left = 176
      Top = 401
      Width = 153
      Height = 25
      Caption = 'Use real compass sensor'
      TabOrder = 11
    end
    object TbCompass: TTrackBar
      Left = 20
      Top = 437
      Width = 157
      Height = 41
      Max = 1000
      Min = -1000
      PageSize = 0
      Frequency = 250
      TabOrder = 12
    end
    object GroupBox2: TGroupBox
      Left = 71
      Top = 208
      Width = 59
      Height = 66
      Caption = 'HEAD/TAIL'
      TabOrder = 13
      object ShHeadLight: TShape
        Left = 16
        Top = 24
        Width = 25
        Height = 25
      end
    end
    object TbMotorL: TTrackBar
      Left = 221
      Top = 424
      Width = 97
      Height = 25
      Max = 100
      Min = -100
      Frequency = 25
      TabOrder = 14
    end
    object TbMotorR: TTrackBar
      Left = 221
      Top = 455
      Width = 97
      Height = 25
      Max = 100
      Min = -100
      Frequency = 25
      TabOrder = 15
    end
    object CbFreezeCompass: TCheckBox
      Left = 224
      Top = 380
      Width = 97
      Height = 17
      Caption = 'Freeze compass'
      TabOrder = 16
    end
    object BtnRebootAtmel: TButton
      Left = 224
      Top = 312
      Width = 73
      Height = 25
      Caption = 'Reboot AVR'
      TabOrder = 17
      OnClick = BtnRebootAtmelClick
    end
  end
  object ListBox1: TListBox
    Left = 0
    Top = 484
    Width = 725
    Height = 151
    Align = alBottom
    ItemHeight = 13
    TabOrder = 1
  end
  object PageControl1: TPageControl
    Left = 0
    Top = 0
    Width = 368
    Height = 484
    ActivePage = TsCourseOverview
    Align = alClient
    TabOrder = 2
    object TsCourseOverview: TTabSheet
      Caption = 'TsCourseOverview'
      ExplicitLeft = 0
      ExplicitTop = 0
      ExplicitWidth = 0
      ExplicitHeight = 0
      object PaintBox1: TPaintBox
        Left = 0
        Top = 0
        Width = 360
        Height = 456
        Align = alClient
        ExplicitLeft = 38
        ExplicitTop = 32
        ExplicitWidth = 368
        ExplicitHeight = 432
      end
    end
    object TsMagneto: TTabSheet
      Caption = 'TsMagneto'
      ImageIndex = 1
      ExplicitLeft = 0
      ExplicitTop = 0
      ExplicitWidth = 0
      ExplicitHeight = 0
      object PaintBox2: TPaintBox
        Left = 0
        Top = 0
        Width = 360
        Height = 456
        Align = alClient
        ExplicitLeft = 24
        ExplicitTop = 16
        ExplicitWidth = 321
        ExplicitHeight = 369
      end
    end
  end
  object Timer1: TTimer
    Interval = 40
    OnTimer = Timer1Timer
    Left = 128
    Top = 128
  end
  object Timer2: TTimer
    Interval = 250
    OnTimer = Timer2Timer
    Left = 148
    Top = 232
  end
end
