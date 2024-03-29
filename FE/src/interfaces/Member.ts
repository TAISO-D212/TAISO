export interface Member {
  email: string,
  pwd?: string,
  name?: string,
  deleteFlag?: boolean,
  accessToken?: string,
  refreshToken?: string,
  error?: MemberError
  
}

export interface MemberError {
  error?: string
}

export interface SignUpInputType {
  email: string,
  pwd: string,
  name: string
}
