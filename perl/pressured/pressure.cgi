#!/usr/bin/perl
use strict;
use warnings;
 
use CGI;
use Net::MQTT::Simple;
use Net::MQTT::Simple::Auth;
use JSON;
use Try::Tiny;
use Data::Dumper qw( Dumper );

my $mqttServer="10.64.9.77";
my $user="";
my $pass="";
my $v;
my $mqtt;
my $q=CGI->new;

# Setup MQTT connection
if ( length($user) && length($pass) ) {
  $mqtt = Net::MQTT::Simple::Auth->new( $mqttServer, $user, $pass );
} else {
  $mqtt = Net::MQTT::Simple->new($mqttServer);
}


my @subs = ( 'petaluma/lab/air' => \&airhandler );
$mqtt->subscribe(@subs);
my $loop=10;
while($loop--) {
  my $r = $mqtt->tick(5);
}

print($q->header(-expires=>'+5s'),"\n");
print <<"EoF";
<!DOCTYPE html>
<html lang="en" >
<head> 
  <title>Petaluma Lab Air</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <meta http-equiv="refresh" content="30" />
</head>
EoF

if (ref($v)) {
  my $airline_press=$v->{'AirlinePressure_PSI'};
  my $ambient_tempC=$v->{'AmbientTemp_C'};
  my $ambient_tempF=ctof($ambient_tempC);
  my $ambient_humidity=$v->{'AmbientHumidity_Percent'};
  my $ambient_pressure=$v->{'AmbientPressure_mb'};
  my $dewpointC=$v->{'AmbientDewPoint_C'};
  my $dewpointF=ctof($dewpointC);
  
  
  print <<"EoF";
<body>
  <center><H1>Petaluma Lab Air</H1></center><p>
  <H2>Compressed Air Pressure: $airline_press psi</H2>
  <H2>Ambient Temp: $ambient_tempC &deg;C, $ambient_tempF &deg;F</H2>
  <H2>Ambient RH: $ambient_humidity &percnt;</H2>
  <H2>Ambient Air pressure: $ambient_pressure mb</H2>
  <H2>Dewpoint: $dewpointC &deg;C, $dewpointF &deg;F</H2>

<p>
This data is available for M2M use via MQTT on hwlin1 (10.64.9.77). No authentication, topic is 
"petaluma/lab/air" returning a json encoded structure like:<br>
<pre>{"AirlinePressure_PSI":114.3,"AmbientHumidity_Percent":32,"AmbientTemp_C":26.65,"AirlineTemp_C":26.4,"AmbientDewPoint_C":8.66,"Time":1623962446,"AmbientPressure_mb":1006.76}</pre>

Currently, data are updated about every 5 sec, and it's published no-retain so it should not keep stale data around.

Full sources and design files are available at <a href="https://www.github.com/mikef5410/USB_PressureSensor">my gihub repo</a>
</body>
</html>
EoF
} else {
  print <<"EoF";
<body>
  An error occurred.
</body>
</html>
EoF
}

sub airhandler {
  my $topic = shift;
  my $val   = shift;
  my $json  = JSON->new->allow_nonref;

  try {
    $v = $json->decode($val);
    $loop=0;
  }
  catch {
  };
}
  
sub ctof {
  my $c = shift;

  return(($c * 9.0/5.0) + 32.0);
}

sub mbtoinHG {
  my $mb = shift;
  return($mb/33.864);
}

sub mbtoPSI {
  my $mb = shift;
  return($mb/68.948);
}
