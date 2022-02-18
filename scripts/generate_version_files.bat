::for change unicode page
chcp 65001
::setting company 
SET author="Burak Şahan"
SET company="Rota Teknik"
SET project_name="F1_CompactController_IOExp_Bootloader"

set get_git_version=git describe --always
for /f %%i in ('%get_git_version%') do set git_version=%%i

..\..\..\APPInfoGenerator\bin\Debug\APPInfoGenerator.exe -projectroot ".." -srcroot "\src" -author %author% -company %company% -projectname %project_name% -gitversion %git_version% -src_outpath "..\Core\src" -hdr_outpath "..\Core\inc" -outputfilename "rt_app_info" -otype DUAL


