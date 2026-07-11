use oxidor_protos::prost::Message;
use oxidor_protos::sat::{CpModelProto, IntegerVariableProto, SatParameters};

#[test]
fn cp_model_proto_roundtrips_through_bytes() {
    let model = CpModelProto {
        name: "roundtrip".to_string(),
        variables: vec![IntegerVariableProto {
            name: "x".to_string(),
            domain: vec![0, 10],
        }],
        ..Default::default()
    };

    let bytes = model.encode_to_vec();
    let decoded = CpModelProto::decode(bytes.as_slice()).unwrap();
    assert_eq!(decoded, model);
}

#[test]
fn sat_parameters_carry_proto2_defaults() {
    let params = SatParameters::default();
    // Unset proto2 optional fields fall back to their declared defaults
    // through the accessor methods.
    assert!(params.max_time_in_seconds() > 1e18);
    assert_eq!(params.num_workers(), 0);

    let tuned = SatParameters {
        max_time_in_seconds: Some(30.0),
        ..Default::default()
    };
    let decoded = SatParameters::decode(tuned.encode_to_vec().as_slice()).unwrap();
    assert_eq!(decoded.max_time_in_seconds(), 30.0);
}
