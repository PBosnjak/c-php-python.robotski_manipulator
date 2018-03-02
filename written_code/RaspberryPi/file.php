<?php
$handle = fopen ("values.txt", "c");
foreach ($_POST as $var){
    fwrite($handle, $var.PHP_EOL);
}

fclose($handle);

if ($handle) header("Location: index.php?sent=1");
else header("Location: index.php?sent=0");
die();




