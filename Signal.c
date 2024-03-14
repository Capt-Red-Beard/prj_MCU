//���� �������� ������ � ������� � �������� ��������

////////////////////////////////////////////////////////////////////////////
////////////////////////       �������� �������   //////////////////////////
////////////////////////////////////////////////////////////////////////////
unsigned short int Signals_Array[32][2][16]=
  {
	{//������ �0 - ������
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �1 (��������� �������)
		{5000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �2 ���������� ������� ����������
		{3000,1500,0,3000,1500,0,3000,1500,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{50,100,300,50,100,300,50,100,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �3 ��������� ����� � �����������
		{2300,0,2600,0,2300,0,2600,0,2300,0,2600,0,2300,0,2600}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �4 ����������� ����� � �����������
		{2300,0,2600,0,2300,0,2600,0,2300,0,2600,0,2300,0,2600}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �5 �������� ������� �����
		{1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �6 ��������� ������� �����
		{1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �7 ������� ����� ��������� ����� �������
		{1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �8 ����� �� ������ ��������� ����� �������
		{1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �9 ������� ����� ��������� ����� ��������
		{1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �10 ����� �� ������ ��������� ����� ��������
		{1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �11 ������ ���������� ���������
		{2000,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �12 ���������� ���������
		{2300,0,2300,0,2300,0,2300,0,2300,0,2300,0,2300,0,2300}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �13 �������� ����  �� ������� ������� 0.5 �
		{2300,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �14 �������� ����  �� ������� ������� 1  �
		{2300,0,2300,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,100,200,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �15 �������� ����  �� ������� ������� 5 �
		{2300,0,2300,0,2300,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,100,200,100,200,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �16 �������  ����  �� ������� ������� 0.5 �
		{2300,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �17 �������  ����  �� ������� ������� 1  �
		{2300,0,2300,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,100,200,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �18 �������  ����  �� ������� ������� 5 �
		{2300,0,2300,0,2300,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{200,100,200,100,200,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �19 (���������� ������� �������� ����������)
		{1500,3000,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{50,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �20 (��������� ������� �������� ����������)
		{3000,1500,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{50,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �21
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �22
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �23
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �24
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �25
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �26
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �27
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �28
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �29
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �30
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	},
	{//������ �31 (������� ����� ���������� � �����)
		{2300,0,3500,0,3500,0,3500,0,0,0,0,0,0,0,0}, //������� �����, 1�� - 65535 �� (���� ������� ����� ����, �� ������������� �����)
		{5000,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0}  //������������ �����, 0 �� - 65535 ��
	}
  };

