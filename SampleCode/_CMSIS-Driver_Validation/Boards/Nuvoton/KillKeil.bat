::rd  /S /Q Project\MDKARM(uV5)\Listings

del *.crf /Q /S
del *.o   /Q /S
del *.d   /Q /S
del *.axf /Q /S
del *.htm /Q /S
del *.sct /Q /S
del *.dep /Q /S
del *.orig /Q /S

del *.lst /Q /S
del *.map /Q /S

del *.uvguix.*   /Q /S
del *.uvgui.*   /Q /S
del JLinkLog.txt /Q /S

del *.iex /s
del *.tra /s
del *.bak /s
del *.ddk /s
del *.edk /s
del *.lnp /s
del *.mpf /s
del *.mpj /s
del *.obj /s
del *.omf /s
::del *.opt /s  ::����??��JLINK��?�m
del *.plg /s
del *.rpt /s
del *.tmp /s
del *.__i /s
del *.i /s
del *.scvd /s
del *.clangd /s
del *.cbuild-run.yml /s
del *.cbuild-idx.yml /s
del *.cbuild-pack.yml /s
del *.cbuild-set.yml /s
del *.Release+ARMCLANG.cbuild.yml /s
del *.Release+GNUC.cbuild.yml /s

:: 設定要刪除的資料夾名稱（可自行擴增）
set FOLDERS=Release release Listings lst Lst Objects Obj obj RTE rte out tmp settings

:: 先刪除當前目錄下的目標資料夾
for %%F in (%FOLDERS%) do (
    if exist "%%F" (
        echo Deleting .\%%F
        rmdir /s /q "%%F"
    )
)

for %%F in (%FOLDERS%) do (
    for /d /r %%D in (%%F) do (
        if exist "%%D" (
            echo Deleting %%D
            rmdir /s /q "%%D"
        )
    )
)
exit
