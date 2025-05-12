use std::env;

fn main() {
    #[cfg(feature = "hyrl")]
    run_tonic();
}

fn run_tonic() {
    let proto_path = env::var("OBSTACLE_AVOIDANCE_APIS")
        .expect("OBSTACLE_AVOIDANCE_APIS not set");

    println!("{}", proto_path);
    let proto_files = vec![
        format!("{}/protos/hyrl_api/obstacle_avoidance.proto", proto_path),
    ];

    tonic_build::configure()
        .build_server(false)
        .build_client(true)
        .compile_protos(&proto_files, &[proto_path])
        .expect("Failed to compile proto files");

    for proto in &proto_files {
        println!("cargo:rerun-if-changed={proto}");
    }
}
