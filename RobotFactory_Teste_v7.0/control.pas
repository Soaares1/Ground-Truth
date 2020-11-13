type
  TControlMode = (cmManual);
  TRobotControls = record
    Fh: double;
    V1, V2: double;
  end;
  
const
    cte_wall_length = 4;		// Comprimento da parede
    cte_180divPI = 57.29577951; // 180/PI
	cte_PIdiv180 = 0.017453292;	// PI/180
	cte_30graus = 0.48236;		// 1-0.51764 -> De -30º a +30º
	cte_exclusion = 0.09; // 3 * std_stdev
  
// Global Variables
var
  irobot, NumRobots: integer;
  t: double;
  ControlMode: TControlMode;
  RobotControls: TRobotControls;	// "Struct" que armazena variáveis de controle do Robô
  CameraControls: TRobotControls;	// "Struct" que armazena variáveis de controle da Câmera
  iLaser: integer;
  NetOutBuf: TUDPBuffer;
  ODOWord1, ODOWord2: word;
  firstRay, lastRay: integer;
  Log: TStringList;
  LogOn: boolean;
  robot_posx, robot_posy, robot_ang, camera_posx, camera_posy, camera_ang: double;
  moving_average_x, moving_average_y, lidar_posx, lidar_posy, lidar_ang: double;
  angulo_alvo, difx, dify: double;
  V1, W1: double;
  cont: integer;
  allowed : array[0..359] of double;
  vetor_x : array[1..5] of double;
  vetor_y : array[1..5] of double;
  angrad, obj_x, obj_y : double;
  atual, minus, flag, ini, fim, pmedio: integer;
  exclusion_cte: double;
  wall_length: double;
  limite: double;
  pix: TRGBAColor; 
  Texto: TStringlist;
  flag_record_data: boolean;



procedure ManualControl(var RC: TRobotControls); // Controle manual do AGV
var V, W: double;
    Vmax, Wmax: double;
begin

	Vmax := 1; Wmax := 0.5;
	V := 0; W := 0;

	if KeyPressed(vk_down) then begin
	V := -1;
	end else if KeyPressed(vk_left) then begin
	W := 1;
	end else if KeyPressed(vk_right) then begin
	W := -1
	end else if KeyPressed(vk_up) then begin
	V := 1;
	end;

	RC.V1 := V * Vmax - W * Wmax;
	RC.V2 := V * Vmax + W * Wmax;

	if KeyPressed(ord('1')) then begin
	RC.Fh := 0.0;
	end else if KeyPressed(ord('2')) then begin
	RC.Fh := 0.0152;
	end else if KeyPressed(ord('3')) then begin
	RC.Fh := 0.04;
	end;
end;

procedure LaserModel(Robot, Laser:Integer; a1, a2, a3, b1, b2 : double);
var	D, N, sigma, Dl : double;
	Laser_scan, Ds : Matrix;
	i : integer;
begin

	flag := 0; ini := 0; fim := 0;
	
	Laser_scan := GetSensorValues(Robot, Laser);

	Ds := Mzeros(MNumRows(Laser_scan),
			   MNumCols(Laser_scan));

	for i:= 0 to MNumRows(Laser_scan)-1 do begin
		D := Mgetv(Laser_scan,i,0);
		Dl := a1 * D * D + a2 * D + a3; 
	
 		// Zera todos os valores que estão fora da zona de aceitação
 		if (Dl >= allowed[i]) then 
			begin
				Dl := 0;
			end; 	    
			
		// Detecção do ponto médio do robô	
{ 		if (Dl <> 0) and (flag = 0) then	// Se o ponto atual for diferente de 0, seta a flag
			begin
				flag := 1;
			end
 		else if (Dl <> 0) and (flag = 1) and (ini = 0) then // Se o ponto atual também for diferente de 0... (Será o segundo ponto consecutivo dif. de zero).
			begin
				ini := i-1;	// O ponto anterior é definido como "início"
			end
		else if (Dl <> 0) and (flag = 1) and (ini <> 0) then  // Se o ponto atual for diferente de zero, marcá-lo como "fim"
			begin
				fim := i;
			end
		else // Se alguma das condições acima falhar, o ponto atual não é referente ao robô. Zerar flag.
			begin
				flag := 0;
			end;	  }
			
			
		if (Dl <> 0) and (flag = 0) and (ini = 0) then // Pode ser o início -> Seta a flag 
			begin
				flag := 1;
			end
		else if (Dl = 0) and (flag = 1) and (ini = 0) then // O ponto anterior não era o início, limpa flag.
			begin
				flag := 0;
			end
		else if (Dl <> 0) and (flag = 1) and (ini = 0) then // O ponto anterior realmente era o início. 
			begin
				ini := i - 1;
			end
		else if (Dl = 0) and (flag = 1) and (ini <> 0) then // O ponto anterior era o final
			begin
				fim := i - 1;
				flag := 0;
			end;

		Msetv(Ds, i, 0, Dl);  // Atribui o valor de D1 à posição i da matriz Ds
  end;

	pmedio := round((fim + ini)*0.5);	// Encontra o ponto médio do robô ########## MUDAR P/ MÉDIA DE N PONTOS
	
	
	// Transformação p/ coordenadas cartesianas
	obj_x := Mgetv(Ds,pmedio,0)*cos((pmedio*(cte_PIdiv180))/2) + 0.10 * cos((pmedio*(cte_PIdiv180))/2);	
	obj_y := Mgetv(Ds,pmedio,0)*sin((pmedio*(cte_PIdiv180))/2) + 0.10 * sin((pmedio*(cte_PIdiv180))/2);
	
	// Média Móvel de 5 elementos
	atual := atual + 1;
	minus := atual - 5;
	
	if (atual > 4) then begin vetor_x[atual - minus] := vetor_x[atual - minus - 1]; vetor_y[atual - minus] := vetor_y[atual - minus - 1]; end;
	if (atual > 3) then begin vetor_x[atual - minus - 1] := vetor_x[atual - minus - 2]; vetor_y[atual - minus - 1] :=  vetor_y[atual - minus - 2];  end;
	if (atual > 2) then begin vetor_x[atual - minus - 2] := vetor_x[atual - minus - 3]; vetor_y[atual - minus - 2] := vetor_y[atual - minus - 3]; end;
	if (atual > 1) then begin vetor_x[atual - minus - 3] := vetor_x[atual - minus - 4]; vetor_y[atual - minus - 3] := vetor_y[atual - minus - 4]; end;
	vetor_x[atual - minus - 4] := obj_x; 
	vetor_y[atual - minus - 4] := obj_y; 
		
	moving_average_x := ( vetor_x[1] + vetor_x[2] + vetor_x[3] + vetor_x[4] + vetor_x[5] ) * 0.2;
	moving_average_y := ( vetor_y[1] + vetor_y[2] + vetor_y[3] + vetor_y[4] + vetor_y[5] ) * 0.2;
	//moving_average_x := obj_x;
	//moving_average_y := obj_y;
			
	SetRCValue(24, 1 ,format('%.3g',[moving_average_x])); 
	SetRCValue(24, 2 ,format('%.3g',[moving_average_y])); 
	SetRCValue(32, 1 ,format('%.3g',[abs(robot_posx-moving_average_x)])); 
	SetRCValue(32, 2 ,format('%.3g',[abs(robot_posy-moving_average_y)])); 
end;

procedure CameraMove(var RC: TRobotControls); // Controle da câmera
var cte, ang_real, dif, ang : double;
begin

	cte := 1.0*(cte_PIdiv180); // Constante de Zona morta 

	// Diferença entre as coordenadas do robô e as coordenadas da câmera
	difx := moving_average_x;
	dify := moving_average_y - 0.10;

	angulo_alvo := ATan2(dify,difx); // Determina o ângulo que a câmera deve "olhar" para encontrar o robô
	
	// Limita o angulo_alvo, i.e., o movimento da câmera entre 0 e PI
	if (angulo_alvo < 0) then begin
		if (angulo_alvo > -PI/2) then begin
				angulo_alvo := 0;
			end
		else
			begin
				angulo_alvo := PI;
		end;
	end; 		
  
    SetRCValue(13, 5 ,format('%.5g',[angulo_alvo*(cte_180divPI)]));
  
	if ((camera_ang > angulo_alvo - cte ) and (camera_ang < angulo_alvo + cte)) then begin // Zona morta 
			W1 := 0; 	
			flag_record_data := true;
		end
	else if (camera_ang < angulo_alvo) then
		begin
			W1 := 0.08;	
		end
	else
		begin
			W1 := - 0.08;
	end;
    
	RC.V1 := - W1;
	RC.V2 :=  W1;

	if KeyPressed(ord('1')) then begin
	RC.Fh := 0.0;
	end 
	else if KeyPressed(ord('2')) then begin
	RC.Fh := 0.0152;
	end 
	else if KeyPressed(ord('3')) then begin
	RC.Fh := 0.04;
	end;
end;

function borda_vertical(const coluna, lin_inicial, lin_final, referencia: integer): integer;
var a, temp, flag1: integer;
begin
	flag1 := 0;
	if lin_inicial < lin_final then begin
		for a := lin_inicial to lin_final do begin	// Busca "espaçada". Para quando encontrar um ponto branco.
			pix := GetCameraPixel(coluna, a);
			if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 0) then begin // Se for encontrado o primeiro pixel branco
				flag1 := 1;
			end	
			else if (pix.red <> pix.green) and (pix.blue <> pix.green) and (pix.red < referencia) and (flag1 = 1) then begin // Se o segundo pixel não for branco o ponto anterior não era a borda
				flag1 := 0;
			end
			else if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 1) then begin // Se for encontrado o segundo pixel branco o ponto anterior realmente era a borda
				a := a - 1;																								
				break;
			end;
		end;
	end
	else begin // lin_inicial > lin_final
		for a := lin_inicial downto lin_final do begin	// Busca "espaçada". Para quando encontrar um ponto branco.
			pix := GetCameraPixel(coluna, a);
			if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 0) then begin // Se for encontrado o primeiro pixel branco
				flag1 := 1;
			end	
			else if (pix.red <> pix.green) and (pix.blue <> pix.green) and (pix.red < referencia) and (flag1 = 1) then begin // Se o segundo pixel não for branco o ponto anterior não era a borda
				flag1 := 0;
			end
			else if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 1) then begin // Se for encontrado o segundo pixel branco o ponto anterior realmente era a borda
				a := a + 1;																								
				break;
			end;
		end;
	end;
	result := a;
end;

function borda_horizontal(const linha, col_inicial, col_final, referencia: integer): integer;
var a, temp, flag1: integer;
begin
	flag1 := 0;
	if col_inicial < col_final then begin
		for a := col_inicial to col_final do begin	// Busca "espaçada". Para quando encontrar um ponto branco.
			pix := GetCameraPixel(a, linha);
			if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 0) then begin // Se for encontrado o primeiro pixel branco
				flag1 := 1;
			end	
			else if (pix.red <> pix.green) and (pix.blue <> pix.green) and (pix.red < referencia) and (flag1 = 1) then begin // Se o segundo pixel não for branco o ponto anterior não era a borda
				flag1 := 0;
			end
			else if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 1) then begin // Se for encontrado o segundo pixel branco o ponto anterior realmente era a borda
				a := a - 1;																								
				break;
			end;
		end;
	end
	else begin
		for a := col_inicial downto col_final do begin	// Busca "espaçada". Para quando encontrar um ponto branco.
			pix := GetCameraPixel(a, linha);
			if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 0) then begin // Se for encontrado o primeiro pixel branco
				flag1 := 1;
			end	
			else if (pix.red <> pix.green) and (pix.blue <> pix.green) and (pix.red < referencia) and (flag1 = 1) then begin // Se o segundo pixel não for branco o ponto anterior não era a borda
				flag1 := 0;
			end
			else if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= referencia) and (flag1 = 1) then begin // Se for encontrado o segundo pixel branco o ponto anterior realmente era a borda
				a := a + 1;																								
				break;
			end;
		end;
	end;
	result := a;
end;

function determina_angulo(const linha, esq, dir, ref: integer): double;
var a, aux, flag_cor, conta_branco, conta_cor: integer;
	raio, num, den, medida, ang: double;
begin
	flag_cor := 0; conta_branco := 0; conta_cor := 0; aux := 0;
	
	raio := round( (dir - esq) * 0.5);
	aux := round(raio*cte_30graus);
	
	for a := round(esq+aux) to round(esq+aux+raio) do begin 
		pix := GetCameraPixel(a, linha);
		if (pix.red = pix.green) and (pix.blue = pix.green) and (pix.red >= ref) then begin	// Branco
			if (flag_cor = 0) then begin
				conta_branco := conta_branco + 1;
			end 
			else begin	// Termina
				break;
			end;
		end
		else begin	// Se for colorido
			conta_cor := conta_cor + 1;
			if (pix.blue > pix.red) and (pix.blue > pix.green) and (flag_cor = 0) then begin 		// Azul
				flag_cor := 1;
			end
			else if (pix.green > pix.red) and (pix.green > pix.blue) and (flag_cor = 0) then begin // Verde
				flag_cor := 2;
			end
			else if (pix.red > pix.green) and (pix.blue > pix.green) and (flag_cor = 0) then begin // Rosa
				flag_cor := 3;
			end
			else if (pix.red > pix.blue) and (pix.green > pix.blue) and (flag_cor = 0) then begin 	// Amarelo
				flag_cor := 4;
			end
			else if (pix.red > pix.blue) and (pix.red > pix.green) and (flag_cor = 0) then begin 	// Vermelho
				flag_cor := 5;
			end
			else if (pix.green > pix.red) and (pix.blue > pix.red) and (flag_cor = 0) then begin 	// Azul claro
				flag_cor := 6;
			end;
		end;			
	end;
		
	medida := conta_branco + conta_cor * 0.5;
		
	if (raio <> 0) then begin
		num := (2*raio*raio - medida*medida);
		den := (2*raio*raio); 
		ang := arccos(num/den) * cte_180divPI;
	end;
	
   	Case flag_cor of
		1: ang := ang - 30;		// Azul
		2: ang := ang - 90;		// Verde
		3: ang := ang - 150;	// Rosa
		4: ang := ang - 210;	// Amarelo
		5: ang := ang - 270;	// Vermelho
		6: ang := ang - 330;	// Azul Claro
	end;   
	
 	SetRCValue(23, 5 ,format('%d',[conta_branco]));
	SetRCValue(24, 5 ,format('%d',[conta_cor]));
	SetRCValue(25, 5 ,format('%.3g',[raio]));
	SetRCValue(26, 5 ,format('%.3g',[medida]));	
	
	result := ang;
end;


procedure Control;
var StrPacket: TStringList;
    txt, s, u: string;
    i: integer;
    rgbacolor: trgbacolor;
    LaserValues, B, Noise, Filter: Matrix;
	z, r, c, dir, esq, cima, baixo, ref, a, pd, step, agv: integer;
	cte, ang_real, dif, ang: double;

begin
	t := t + ScriptPeriod();
	// Coordenadas e ângulo - Robô AGV (p/ comparação)
	robot_posx := -GetRobotX(0);
	robot_posy := 2-GetRobotY(0);
	robot_ang := GetRobotTheta(0)+PI;
	SetRCValue(28, 1 ,format('%.3g',[robot_posx]));
	SetRCValue(28, 2 ,format('%.3g',[robot_posy]));
	SetRCValue(28, 3 ,format('%.4g',[robot_ang*(cte_180divPI)]));
	// Coordenadas e ângulo - Câmera (p/ comparação)
	camera_posx := GetRobotX(1);
	camera_posy := GetRobotY(1);
	camera_ang := GetRobotTheta(1);
	SetRCValue(13, 1 ,format('%.3g',[camera_posx]));
	SetRCValue(13, 2 ,format('%.3g',[camera_posy]));
	SetRCValue(13, 3 ,format('%.3g',[camera_ang*(cte_180divPI)]));
	// Coordenadas - LIDAR
	lidar_posx := GetRobotX(2);
	lidar_posy := GetRobotY(2);
	SetRCValue(16, 1 ,format('%.3g',[lidar_posx]));
	SetRCValue(16, 2 ,format('%.3g',[lidar_posy]));
	SetRCValue(16, 3 ,'-----');
  
	if iLaser >= 0 then begin
		LaserModel(2, iLaser, 0, 1, 0, 0.001523985, 1.3102842636); // O LIDAR corresponde ao robô 2
	end;
   
	if RCButtonPressed(2, 9) then begin // Start Log
		if log <> nil then Log.free;
		Log := TStringList.create;
		LogOn := true;
	end;
  
	if RCButtonPressed(2, 10) then begin // Stop Log
		LogOn := false;
	end;
  
	if RCButtonPressed(2, 11) then begin // Save Log
		if log <> nil then Log.savetofile(GetRCText(2, 12));
	end;
  
	if LogOn then begin
	// 0 ticks loc_age x y theta enc_age enc1 enc2 laser_age Laser
		for i := firstRay to LastRay do begin
			s := s + format('%.5g; ', [Mgetv(LaserValues, i, 0)]);
		end;
		Log.add(s);
	end;

    ControlMode := cmManual;
	ManualControl(RobotControls);
	CameraMove(CameraControls);
		
	ref := 100;
	
	// Formato (coluna,linha) -> 240 linhas, 320 colunas.
	// Encontra a linha referente ao meio do robô
	cima := borda_vertical(160, 0, 240, ref);
	baixo := borda_vertical(160, 230, 0, ref);
	r := round ((baixo + cima) * 0.5 ); // Linha referente ao meio do robô
	
	z := round ((cima + r) * 0.5); // Localizada entre a parte de cima do robô e o meio
	
	// Encontra a coluna referente ao meio do robô
	esq := borda_horizontal(z, 0, 320, ref);
	dir := borda_horizontal(z, 320, 0, ref);
	c := round ((dir + esq) * 0.5 ); // Coluna referente ao meio do robô
	
	SetRCValue(19, 5 ,format('%d',[cima]));
	SetRCValue(21, 5 ,format('%d',[baixo]));
	SetRCValue(20, 4 ,format('%d',[esq])); 
	SetRCValue(20, 6 ,format('%d',[dir])); 
	//SetRCValue(23, 4 ,format('%d',[r])); 
	//SetRCValue(23, 5 ,format('%d',[c])); 
	
	ang := determina_angulo(r, esq, dir, ref);
	ang := ang + angulo_alvo*(cte_180divPI) - 90;
	ang_real := robot_ang*(cte_180divPI);
	
 	if (ang < 0) then begin
		ang := ang + 360;
	end
	else if (ang >= 360) then begin
		ang := ang - 360;
	end; 	
		
	SetRCValue(24, 3 ,format('%.4g',[ang])); 
	dif := abs(ang - ang_real);	// Erro absoluto
	if (dif >= 350) then begin dif := abs(360 - dif); end;
	SetRCValue(32, 3 ,format('%.4g',[dif]));
	
	if (flag_record_data = true) then begin
		u := u + format('%.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g;', [obj_x, obj_y, moving_average_x, moving_average_y, ang, robot_posx , robot_posy, ang_real, abs(robot_posx-moving_average_x), abs(robot_posy-moving_average_y), dif]);
		Texto.add(u); // Adicionar 
		Texto.savetofile('data.txt'); // Salvar }
	end;
	
	
	SetMotorControllerState(0, 0, true);
    SetMotorControllerState(0, 1, true);
    SetAxisSpeedRef(0, 0, 10*RobotControls.V1);
    SetAxisSpeedRef(0, 1, 10*RobotControls.V2);
    SetAxisPosRef(0, 2, RobotControls.Fh);
	
	SetMotorControllerState(1, 0, true);
    SetMotorControllerState(1, 1, true);
    SetAxisSpeedRef(1, 0, 10*CameraControls.V1);
    SetAxisSpeedRef(1, 1, 10*CameraControls.V2);
    SetAxisPosRef(1, 2, CameraControls.Fh);
end;

procedure Initialize;
begin
	iLaser := GetSensorIndex(2, 'ranger2d');
	firstRay := 0;
	lastRay := 359;
	t := 0;
	ControlMode := cmManual;
	Log := nil;
	Texto := nil;
	LogOn := false;
	flag_record_data := false; 
	V1 := 0;
	W1 := 0;
	SetRCValue(2, 2, 'Manual'); 
	SetRCValue(9, 1, 'robot_posx'); SetRCValue(9, 2, 'robot_posy'); SetRCValue(9, 3, 'robot_ang');
	SetRCValue(12, 1, 'cam_posx'); SetRCValue(12, 2, 'cam_posy'); SetRCValue(12, 3, 'cam_ang');
	SetRCValue(15, 1, 'lidar_posx'); SetRCValue(15, 2, 'lidar_posy'); SetRCValue(15, 3, 'lidar_ang');
	SetRCValue(12, 5, 'ang_alvo');
	SetRCValue(22, 1, 'Estimado'); SetRCValue(23, 1, 'x'); SetRCValue(23, 2, 'y');  SetRCValue(23, 3  ,'angulo');
	SetRCValue(26, 1, 'Real'); SetRCValue(27, 1, 'x'); SetRCValue(27, 2, 'y');  SetRCValue(27, 3  ,'angulo');
	SetRCValue(30, 1, 'Erro abs'); SetRCValue(31, 1, 'x'); SetRCValue(31, 2, 'y');  SetRCValue(31, 3  ,'angulo');
	SetRCValue(23, 4 ,'conta_branco');
	SetRCValue(24, 4 ,'conta_cor');
	SetRCValue(25, 4 ,'raio');
	SetRCValue(26, 4 ,'medida');
	SetRCValue(34, 1 ,'AGV');
		
	Texto := TStringList.create;
	
{ 	for cont := 1 to 5 do begin
		vetor_x(cont) := 0.0;
		vetor_y(cont) := 0.0;
	end; }
	
    
	limite := cte_wall_length - cte_exclusion; // Zona de aceitação = Comprimento da parede - Constante de exclusão
  
	// Calcula a zona de aceitação
    for cont := 0 to 359 do begin
		angrad := (cont/2)*(cte_PIdiv180); // Como a varredura do LIDAR vai de 0 a 180º c/ incr. de 0.5º é necessário dividir por 2
		if cont < 90 then // de 0 a PI/4
			begin
				allowed[cont] := limite/cos(angrad); 
			end
		else if (cont >= 90) and (cont < 270) then // de PI/4 a 3PI/4
			begin
				allowed[cont] := limite/sin(angrad);
			end
		else	// de 3PI/4 a PI
			begin
				allowed[cont] := -limite/cos(angrad);
			end;
   end; 
   
end;
