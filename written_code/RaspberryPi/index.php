<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>UGRS</title>
    <link rel="stylesheet" type="text/css" href="style.css">
</head>
<body>
<form action="/ugrs/file.php" method="post">
    Motor 1:<br>
    <input type="range" name="m1" value="50" oninput="updateTextInput1(this.value)" >
    <textarea class="text" id="m1_input" disabled rows="1">50</textarea><br>
    Motor 2:<br>
    <input type="range" name="m2" value="20" oninput="updateTextInput2(this.value)">
    <textarea class="text" id="m2_input" disabled rows="1">20</textarea><br>
    Motor 3:<br>
    <input type="range" name="m3" value="0" oninput="updateTextInput3(this.value)">
    <textarea class="text" id="m3_input" disabled rows="1">0</textarea><br>
    Motor 4:<br>
    <input type="range" name="m4" value="0" oninput="updateTextInput4(this.value)">
    <textarea class="text" id="m4_input" disabled rows="1">0</textarea><br>
    Motor 5:<br>
    <input type="range" name="m5" value="0" oninput="updateTextInput5(this.value)">
    <textarea class="text" id="m5_input" disabled rows="1">0</textarea><br>
    Motor 6:<br>
    <input type="range" name="m6" value="0" oninput="updateTextInput6(this.value)">
    <textarea class="text" id="m6_input" disabled rows="1">0</textarea><br><br>
    <button type="submit" class="btn btn-default">Pokreni</button>
</form>

<?php

if(!empty($_GET)) {
 	if ($_GET['sent']==1) echo "<p>Vrijednosti uspješno poslane!</p>";
	if ($_GET['sent']==0) echo "<p>Došlo je do greške prilikom slanja!</p>";
}
?>
</body>
</html>



<script>
    function updateTextInput1(val) {
        document.getElementById('m1_input').value=val;
    }
    function updateTextInput2(val) {
        document.getElementById('m2_input').value=val;
    }
    function updateTextInput3(val) {
        document.getElementById('m3_input').value=val;
    }
    function updateTextInput4(val) {
        document.getElementById('m4_input').value=val;
    }
    function updateTextInput5(val) {
        document.getElementById('m5_input').value=val;
    }
    function updateTextInput6(val) {
        document.getElementById('m6_input').value=val;
    }
</script>