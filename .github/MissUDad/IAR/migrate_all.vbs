Option Explicit

Dim fso, file, listFile, line, app, shell, cmd
Set fso = CreateObject("Scripting.FileSystemObject")
Set shell = CreateObject("WScript.Shell")

Const IAR_EXE = "C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.4\\common\bin\IarIdePm.exe"

Set listFile = fso.OpenTextFile("list.txt", 1)

Do Until listFile.AtEndOfStream
    line = Trim(listFile.ReadLine)
    If line <> "" Then
        WScript.Echo "Opening: " & line

        ' open project
        shell.Run """" & IAR_EXE & """ """ & line & """"
        WScript.Sleep 5000   ' wait for GUI load

        ' handle upgrade dialog if exists
        On Error Resume Next
        shell.AppActivate "Project Conversion"
        If Err.Number = 0 Then
            shell.SendKeys "{ENTER}"  ' Yes
            WScript.Sleep 3000
        End If
        On Error GoTo 0

        ' wait for full load
        WScript.Sleep 3000

        ' Save the migrated project
        shell.SendKeys "^s"
        WScript.Sleep 2000

        ' Close IAR
        shell.SendKeys "%{F4}"
        WScript.Sleep 2000
    End If
Loop

listFile.Close

WScript.Echo "All projects migrated."
