//address of running sprinkler
const ADDR = 4;

function initWebsocket(page){
  $('#message').show();
  $('#message').html('Connecting...');
  $('#controls').hide();
  
  var url;
  var host = location.hostname;
  if (location.protocol === 'https:') {
    url = 'wss://' + host + location.pathname.substring(0,location.pathname.indexOf('/',1)) + '_ws_/' + page;
  }
  else {
    if (host == 'localhost')
      host = '192.168.1.' + ADDR;
	
    var port = location.port;
    if (port === '')
      port = 81;
    else
      port = parseInt(port) + 1;

    url = 'ws://' + host + ':' + port + '/' + page;
  }
  
  console.log('ws url',url);
  
  socket = new WebSocket(url);
  socket.onopen = function(){
    console.log('onopen');
    $('#message').hide();
    $('#controls').show();
  }
  socket.onmessage = function(msg){
    message(msg);
  }
  socket.onclose = function(){
    console.log('onclose');
    $('#message').show();
    $('#message').html('Disconnected');
    $('#controls').hide();
  }
}