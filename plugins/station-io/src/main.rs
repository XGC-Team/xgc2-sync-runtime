use std::env;
use std::path::Path;

fn main() {
    let mut args = env::args().skip(1);
    let socket = args.next();
    let kind = args.next();
    let value = args.next();
    let result = match (socket, kind.as_deref(), value) {
        (Some(socket), Some("command"), Some(token)) => {
            station_io::submit_command(Path::new(&socket), &token)
        }
        (Some(socket), Some("mission-file"), Some(path)) => {
            station_io::submit_mission_file(Path::new(&socket), Path::new(&path))
        }
        _ => Err("usage: station-io-cmd SOCKET command TOKEN | mission-file FILE".into()),
    };
    if let Err(e) = result {
        eprintln!("{e}");
        std::process::exit(1);
    }
}
