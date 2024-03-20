import { createBrowserRouter } from 'react-router-dom';
import { LoginPage } from './pages/login/LoginPage.tsx';

export const router = createBrowserRouter([
  {path: '/', element: <LoginPage />},
]);
