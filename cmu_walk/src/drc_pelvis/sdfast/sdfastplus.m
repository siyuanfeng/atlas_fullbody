function sdfastplus(name)

% sddir = 'C:\Documents and Settings\bstephe1\My Documents\Matlab\sdfastplus_old';
% prevwd = pwd;
% cd(sddir);

%updated soa 12/23/08


%% STRIP THE DIFFERENT PARTS FROM THE DYNAMICS FILE

fid = fopen(strcat(name,'_d.c'),'r');


if( fid == -1 )
    disp( 'Failed to open dynamics c source' );
    return;
end

DESC = parse_between(fid,'/*','*/');
STRUCTS = parse_between(fid,'typedef','sdgtopo_t');
DEFINES = parse_keys(fid,'sdgtopo');
STRUCTS = cellmerge(STRUCTS,parse_between(fid,'typedef','sdginput_t'));
DEFINES = cellmerge(DEFINES,parse_keys(fid,'sdginput'));
STRUCTS = cellmerge(STRUCTS,parse_between(fid,'typedef','sdgstate_t'));
DEFINES = cellmerge(DEFINES,parse_keys(fid,'sdgstate'));
STRUCTS = cellmerge(STRUCTS,parse_between(fid,'typedef','sdglhs_t'));
DEFINES = cellmerge(DEFINES,parse_keys(fid,'sdglhs'));
STRUCTS = cellmerge(STRUCTS,parse_between(fid,'typedef','sdgrhs_t'));
DEFINES = cellmerge(DEFINES,parse_keys(fid,'sdgrhs'));
STRUCTS = cellmerge(STRUCTS,parse_between(fid,'typedef','sdgtemp_t'));
DEFINES = cellmerge(DEFINES,parse_keys(fid,'sdgtemp'));

CONSTRUCTOR = parse_between(fid,'sdgtopo_t sdgtopo','};');
CONSTRUCTOR = cellmerge(CONSTRUCTOR,parse_between(fid,'sdginput_t sdginput','};'));

FUNCTIONS = parse_between(fid,'void sdinit','*gentm =');
FUNCTIONS{end+1} = '}';

fclose(fid);

%% STRIP THE DIFFERENT PARTS FROM THE ANALYSIS FILE
fid = fopen(strcat(name,'_s.c'),'r');
 if (fid~=-1)
     FUNCTIONS{end+1} = '// start of analysis routines';
     FUNCTIONS = cellmerge(FUNCTIONS,parse_between(fid,'void sdposfunc','*time = ttime;'));
     FUNCTIONS = cellmerge(FUNCTIONS,parse_between( fid, '*err = vintgerr', '*time = ttime' ));
     FUNCTIONS{end+1} = '}';
 end
fclose(fid);

%% STRIP SDLIB for local copies
fid = fopen('sdlib.c');
if( fid~=1)
    STRUCTS = cellmerge(STRUCTS,parse_between( fid, 'typedef struct {', 'sdgerror_t;' ) );
    CONSTRUCTOR = cellmerge(CONSTRUCTOR,parse_between( fid, 'sdgerror_t', '};' ) );
    
    FUNCTIONS{end+1} = '// start of sdlib routines';
    NEW_FUNCS = parse_between( fid, 'void sdprerrmsg(FILE *fnum,', '*serno = 30123;' );
    
    %Some of the sdlib local variable names get shadowed by the defines
    %from the dynamics file.  we need to make local aliases for these names
    %to avoid the colission
    for i=1:length(NEW_FUNCS)
        NEW_FUNCS{i} = strrep( NEW_FUNCS{i}, 'qraux', 'local_qraux' );
        NEW_FUNCS{i} = strrep( NEW_FUNCS{i}, 'jpvt', 'local_jpvt' );
        NEW_FUNCS{i} = strrep( NEW_FUNCS{i}, 'temp', 'local_temp' );
    end
    
    FUNCTIONS = cellmerge(FUNCTIONS, NEW_FUNCS);
    FUNCTIONS{end+1} = '}';
end

% fix the structs
for i=1:length(STRUCTS)
    STRUCTS{i} = strrep(STRUCTS{i},'_t',strcat('_',name,'_t'));
end

%fix the functions
i=1;
while 1
    temp = FUNCTIONS{i};
    if length(temp)>1
    if strcmp(temp(1:2),'sd')
        FUNCTIONS{i-1} = [FUNCTIONS{i-1},temp];
        FUNCTIONS{i} = '';
    end
    end
    i=i+1;
    if i>length(FUNCTIONS)
        break;
    end
end


%make some fixes to handle callbacks through sdlib local members
%we need to make the function pointer prototype explicit
for i=1:length(FUNCTIONS)
    FUNCTIONS{i} = strrep(FUNCTIONS{i},'void sdroot(int (*func)(),', sprintf( 'void sdroot(void (%s::*func)(double *, double *, double *),', name ) ); 
    FUNCTIONS{i} = strrep(FUNCTIONS{i},'void sdvinteg(int (*func)(),', sprintf( 'void sdvinteg(void (%s::*func)(double, double *, double *, double *, int *),' , name ) );
    FUNCTIONS{i} = strrep(FUNCTIONS{i},'void sdfinteg(int (*func)(),', sprintf( 'void sdfinteg(void (%s::*func)(double, double *, double *, double *, int *),' , name ) );
    FUNCTIONS{i} = strrep(FUNCTIONS{i},'void sdadjvars(int (*func)(),', sprintf( 'void sdadjvars(void (%s::*func)(double *, double *, double * ),' , name ) );
    FUNCTIONS{i} = strrep(FUNCTIONS{i},'void sdcalcjac(int (*func)(),', sprintf( 'void sdcalcjac(void (%s::*func)(double *, double *, double * ),' , name ) );
    FUNCTIONS{i} = strrep(FUNCTIONS{i},'void sdrk4m(int (*func)(),', sprintf( 'void sdrk4m(void (%s::*func)(double, double *, double *, double *, int* ),' , name ) );
    
    
    if( ~isempty(strfind( FUNCTIONS{i}, 'void sdmassmat(' ) ) )
        mat_dim = sscanf(FUNCTIONS{i}, 'void sdmassmat(double mmat[%d][%d])' );
        FUNCTIONS{i} = strrep(FUNCTIONS{i},sprintf('mmat[%d][%d]',mat_dim(1),mat_dim(2)),'*mmat');
    end
end

% FUNCTIONS{end+1} = sprintf( 'void %s::massmat( double *mat ) {', name );
% 
% FUNCTIONS{end+1} = sprintf( ' sdmassmat( (double (*)[%d]) mat );\n}', mat_dim(2) );

%% get the function declarations
DECLARES={};
for i=1:length(FUNCTIONS)
    temp = FUNCTIONS{i};
    if (~isempty(strfind(temp,'void sd')) || ~isempty(strfind(temp,'int sd')) || ~isempty(strfind(temp,'double sd')))
        j=i;
        if ~isempty(strfind(temp,'void sdposfunc('))
            DECLARES{end+1} = '// analysis routines';
        end
        while(strcmp(temp(end),','))
            temp = strcat(temp,FUNCTIONS{j+1});
            j=j+1;
        end
        DECLARES{end+1} = [temp,';'];
    end
       
end

%% CREATE THE .H FILE
fih = fopen('template2_h','r');
foh = fopen(strcat(name,'.h'),'w');

while 1
    tline = fgetl(fih);
    if ~ischar(tline),   break,   end  
    % fix structs
    tline = strrep(tline,'_t',strcat('_',name,'_t'));
    if (parse_and_replace(tline,foh,'NAME',name))
        continue;
    end
    if (parse_and_replace(tline,foh,'STRUCTS',STRUCTS))
        continue;
    end
    if (parse_and_replace(tline,foh,'DECLARES',DECLARES))
        continue;
    end
    fprintf(foh,'%s\n',tline);
end

fclose(fih);
fclose(foh);

%% CREATE THE .CPP FILE

fic = fopen('template2_cpp','r');
foc = fopen(strcat(name,'.cpp'),'w');

while 1
    tline = fgetl(fic);
    if ~ischar(tline),   break,   end  
    % fix structs
    tline = strrep(tline,'_t',strcat('_',name,'_t'));
    if (parse_and_replace(tline,foc,'NAME',name))
        continue;
    end
    if (parse_and_replace(tline,foc,'PREDEFS',DEFINES))
        continue;
    end
    if ~isempty(strfind(tline,'CONSTRUCTOR'))
        % print the constructor
        for i=1:length(CONSTRUCTOR)
            temp = CONSTRUCTOR{i};
            temp = strrep(temp,'sdgtopo_t sdgtopo',strcat('sdgtopo_',name,'_t sdgtopo_temp'));
            temp = strrep(temp,'sdginput_t sdginput',strcat('sdginput_',name,'_t sdginput_temp'));
            temp = strrep(temp,'sdgerror_t sdgerror',strcat('sdgerror_',name,'_t sdgerror_temp'));
            fprintf(foc,'%s\n',temp);
        end
        fprintf(foc,'sdgtopo = sdgtopo_temp;\nsdginput = sdginput_temp;\nsdgerror = sdgerror_temp;\n');
        continue;
    end
    if ~isempty(strfind(tline,'FUNCTIONS'))
        % print the public functions
        for i=1:length(FUNCTIONS)
            temp = FUNCTIONS{i};
            temp = strrep(temp,'void sd',['void ',name,'::sd']);
            temp = strrep(temp,'double sd',['double ',name,'::sd']);
            temp = strrep(temp,'int sd',['int ',name,'::sd']);
            % some fixes
            temp = strrep(temp,'sdroot(sdstatfunc',['sdroot(&',name,'::sdstatfunc']);
            temp = strrep(temp,'sdroot(sdstdyfunc',['sdroot(&',name,'::sdstdyfunc']);
            temp = strrep(temp,'sdvinteg(sdmotfunc',['sdvinteg(&',name,'::sdmotfunc']);            
            if ~isempty(strfind(temp,'sdldudcomp'))
                temp = strrep(temp,'workss,works,mm,mlo','(double*)workss,works,(double*)mm,(double*)mlo');
            end
            if ~isempty(strfind(temp,'sdldubslv'))
                temp = strrep(temp,'mlo','(double*)mlo');
            end             
            if ~isempty(strfind(temp,'sdldubsl'))
                temp = strrep(temp,'mlo','(double*)mlo');
            end
            if ~isempty(strfind(temp,'sdqrdcomp'))
                temp = strrep(temp,'ww','(double*)ww');
            end
            if ~isempty(strfind(temp,'sdqrbslv'))
                temp = strrep(temp,'ww','(double*)ww');
            end
            if ~isempty(strfind(temp,'double mmat['))
                temp = strrep(temp,'mmat[','*mmat)//');
            end
            if ~isempty(strfind(temp,'mmat[i][j]'))
                temp = strrep(temp,'mmat[i][j]','mmat[i*ndof + j]');
            end            
            if ~isempty(strfind(temp,'mmat[j][i]'))
                temp = strrep(temp,'mmat[j][i]','mmat[j*ndof + i]');
            end
            if ~isempty(strfind(temp,' func('))
                temp = strrep(temp,'func','(this->*func)');
            end
            if ~isempty(strfind(temp,'sdfinteg(sdmotfunc'))
                temp = strrep(temp,'sdmotfunc',sprintf('&%s::sdmotfunc',name));
            end
            fprintf(foc,'%s\n',temp);
        end
        continue;
    end
    fprintf(foh,'%s\n',tline);
end

fclose(fic);
fclose(foc);
%fclose(foh);

%% create the mex model interface file
% fim = fopen('template_mex.cpp','r');
% fom = fopen(strcat(name,'_mex.cpp'),'w');
% 
% while 1
%     tline = fgetl(fim);
%     if ~ischar(tline),   break,   end  
%     if (parse_and_replace(tline,fom,'NAME',name))
%         continue;
%     end
%     fprintf(fom,'%s\n',tline);
% end
% 
% fclose(fim);
% fclose(fom);

%Generate the model_helper file
% figc = fopen( '../sdfast_generation/model_helper_template.h','r' );
% fogc = fopen( 'model_helper.cpp', 'w' );
% 
% while 1
%      tline = fgetl(figc);
%      if ~ischar(tline),   break,   end  
%      if (parse_and_replace(tline,fogc,'MODEL_GEN_CODE',mgc))
%          continue;
%      end
%      fprintf(fogc,'%s\n',tline);
% end
 
 %eval(strcat('make_',name));
 
%fclose(figc);
%fclose(fogc);

% %% create the mex make file
% fim = fopen('make_template.m','r');
% fom = fopen(strcat('make_',name,'.m'),'w');
% 
% while 1
%     tline = fgetl(fim);
%     if ~ischar(tline),   break,   end  
%     if (parse_and_replace(tline,fom,'NAME',name))
%         continue;
%     end
%     fprintf(fom,'%s\n',tline);
% end
% 
% %eval(strcat('make_',name));
% 
% fclose(fim);
% fclose(fom);
% 


%% finished
% if ~isequal(sddir,prevwd)
% copyfile([name,'.cpp'],prevwd);
% copyfile([name,'.h'],prevwd);
% copyfile(['sdlib.h'],prevwd);
% cd(prevwd);
% end

%disp(['Finished!  Run make_',name,'.m to compile the mex model function']);

function out = isfound(str,val)
out = ~isempty(strfind(str,val));

function C = parse_keys(fid,str)
C={};
while 1 
    tline = fgetl(fid);
    if ~isfound(tline,str),  break, end
    C{end + 1} = tline;
end

function C = parse_between(fid,startstr,endstr)
C={};
found=0;
while 1
    tline = fgetl(fid);
    if ~ischar(tline),   break,   end 
    if (~found && ~isempty(strfind(tline,startstr)))
       found=1;
%        startstr
    end
    if found
        C{end+1} = tline;
        if ~isempty(strfind(tline,endstr)), break, end
    end
end
if ~found
    C = -1;
end

function x = parse_and_replace(tline,fod,key,val)
    x=0;
    if ~isempty(strfind(tline,key))
        if iscell(val)
            for i=1:length(val)
                fprintf(fod,'%s\n',val{i});
            end
        else
            fprintf(fod,'%s\n',strrep(tline,key,val));
        end
        x = 1;
    end

function C = cellmerge(A,B)
    C = cell(1,length(A) + length(B));
    for i=1:length(A)
        C{i} = A{i};
    end
    for i = 1:length(B)
        C{length(A)+i} = B{i};
    end